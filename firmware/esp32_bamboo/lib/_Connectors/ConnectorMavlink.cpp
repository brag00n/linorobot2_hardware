#include "ConnectorMavlink.hpp"

// Garde de compilation : PlatformIO (LDF) compile TOUS les .cpp de lib/_Connectors/
// quel que soit l'env actif. Sans ENABLE_MAVLINK (ex. env trames binaires), le
// chemin d'inclusion du dialecte n'est pas fourni -> tout le corps est neutralise
// et cette unite de traduction devient vide. Meme motif que la STM32 (mav_protocol.c).
#ifdef ENABLE_MAVLINK

#include <math.h>

// Dialecte MAVLink v2 genere (header-only, vendore dans firmware/_common). On ne
// definit PAS MAVLINK_USE_CONVENIENCE_FUNCTIONS : on packe dans un
// mavlink_message_t puis on serialise a la main vers l'UART (Serial.write),
// exactement comme le template STM32 (mav_protocol.c).
#include "bamboo/mavlink.h"

// Geometrie roue : memes placeholders que ConnectorSerialFrame tant que la
// geometrie reelle de la Waveshare n'est pas mesuree (cf. caveat calibration).
#include "config.h"

#ifndef COUNTS_PER_REV1
#define COUNTS_PER_REV1 2114
#endif
#ifndef WHEEL_DIAMETER
#define WHEEL_DIAMETER 0.8
#endif
#ifndef LR_WHEELS_DISTANCE
#define LR_WHEELS_DISTANCE 1.3
#endif

#define WSM_CPR      ((float)(COUNTS_PER_REV1))
#define WSM_CIRC_MM  (((float)WHEEL_DIAMETER) * PI * 1000.0f)       // circonference en mm
#define WSM_APB_MM   (((float)LR_WHEELS_DISTANCE) * 0.5f * 1000.0f) // demi-voie/empattement en mm
#define WSM_CAR_TYPE (4.0f)                                         // FOURWHEEL (TB6612 differentiel)

// Adressage MAVLink (cf. ConnectorMavlink.hpp / mav_protocol.h STM32).
#define MAV_SYS_ID_ESP32   (2)
#define MAV_COMP_ID        (1)   // MAV_COMP_ID_AUTOPILOT1

// ======================================================================= //
//  Etat TX/RX : un seul connectorROS -> etat de fichier (comme la STM32).  //
//  L'ESP32 est mono-thread (loop), aucun garde de concurrence necessaire.  //
// ======================================================================= //
static mavlink_message_t s_tx_msg;
static uint8_t           s_tx_buf[MAVLINK_MAX_PACKET_LEN];

static mavlink_message_t s_rx_parse;    // etat de parsing
static mavlink_status_t  s_rx_status;
static mavlink_message_t s_rx_ready;    // dernier message complet a router
static bool              s_rx_flag = false;

// --- table de parametres (identique a la STM32 pour un codec hote uniforme) ---
// L'ESP32 n'a qu'un jeu de gains PID partage par les 4 moteurs : MOTn_Kx sont
// tous adosses au meme Pid_ (comme l'echo unique de ConnectorSerialFrame).
enum {
    P_MOT1_KP, P_MOT1_KI, P_MOT1_KD,
    P_MOT2_KP, P_MOT2_KI, P_MOT2_KD,
    P_MOT3_KP, P_MOT3_KI, P_MOT3_KD,
    P_MOT4_KP, P_MOT4_KI, P_MOT4_KD,
    P_YAW_KP,  P_YAW_KI,  P_YAW_KD,
    P_WHEEL_CPR, P_WHEEL_CIRC, P_WHEEL_APB,
    P_CAR_TYPE,
    PARAM_COUNT
};

static const char * const s_param_name[PARAM_COUNT] = {
    "MOT1_KP", "MOT1_KI", "MOT1_KD",
    "MOT2_KP", "MOT2_KI", "MOT2_KD",
    "MOT3_KP", "MOT3_KI", "MOT3_KD",
    "MOT4_KP", "MOT4_KI", "MOT4_KD",
    "YAW_KP",  "YAW_KI",  "YAW_KD",
    "WHEEL_CPR", "WHEEL_CIRC", "WHEEL_APB",
    "CAR_TYPE",
};

// --- cycle de vie ---
bool ConnectorMavlink::initAgent(const connectorTimerCallbak_t ptimerCallback,
                                 connectorTwistCallbak_t ptwistCallback,
                                 connectorJointCallbak_t pJointCallback,
                                 connectorPidCallbak_t pPidCallback) {
    timerCallback_ = ptimerCallback;
    twistCallback_ = ptwistCallback;
    jointCallback_ = pJointCallback;
    pidCallback_   = pPidCallback;
    s_rx_flag = false;
    return true; // Serial deja demarre par setup() ; pas d'agent externe
}

bool ConnectorMavlink::isAvailable() { return true; }

// Toujours vrai en mode filaire (pas d'agent) : evite un fullStop() intempestif.
// Le fail-safe "pas de cmd_vel depuis 500 ms" de moveBase() reste le garde-fou.
bool ConnectorMavlink::pingAgent(int, int) { return true; }

bool ConnectorMavlink::listenAgent(long) {
    // 1) vider le flux entrant et router les messages complets (CRC OK).
    while (Serial.available() > 0) {
        parseByte((uint8_t)Serial.read());
    }
    uint32_t now = millis();
    // 2) HEARTBEAT ~1 Hz (presence, type rover, etat).
    if (now - lastHeartbeat_ >= 1000) {
        lastHeartbeat_ = now;
        sendHeartbeat();
    }
    // 3) tick de controle ~10 Hz : moveBase() + publishData() via le callback
    //    firmware, qui declenche l'auto-emission de la telemetrie.
    if (now - lastTick_ >= 100) {
        lastTick_ = now;
        if (timerCallback_ != NULL) {
            timer_t t; t.impl = NULL;
            timerCallback_(&t, 0, "ConnectorMavlink");
        }
    }
    return true;
}

// --- emission de bas niveau ---
void ConnectorMavlink::sendMessage() {
    uint16_t len = mavlink_msg_to_send_buffer(s_tx_buf, &s_tx_msg);
    Serial.write(s_tx_buf, len);
}

void ConnectorMavlink::sendHeartbeat() {
    mavlink_msg_heartbeat_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                               MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
    sendMessage();
}

// --- emission des metriques (interface Connector) ---

void ConnectorMavlink::publishOdom(Odometry::Odometry_data odom) {
    // BAMBOO_WHEEL_STATE : vx/vy en m/s, wz en rad/s (unites SI, pas de mise a l'echelle).
    mavlink_msg_bamboo_wheel_state_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                                        (uint32_t)millis(),
                                        (float)odom.twist_twist_linear_x,
                                        (float)odom.twist_twist_linear_y,
                                        (float)odom.twist_twist_angular_z);
    sendMessage();
}

void ConnectorMavlink::publishImu(IMUInterface::Imu_t m) {
    // Quaternion -> angles d'Euler (roll/pitch/yaw en rad), rates depuis le gyro.
    const float qx = m.orientation.x, qy = m.orientation.y, qz = m.orientation.z, qw = m.orientation.w;
    const float roll  = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    const float sinp  = 2.0f * (qw * qy - qz * qx);
    const float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(HALF_PI, sinp) : asinf(sinp);
    const float yaw   = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    mavlink_msg_attitude_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                              (uint32_t)millis(), roll, pitch, yaw,
                              m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z);
    sendMessage();
}

void ConnectorMavlink::publishBattery(Battery::Battery_t b) {
    long mv = lroundf(b.voltage * 1000.0f);
    battMv_ = (uint16_t)(mv < 0 ? 0 : (mv > 65535 ? 65535 : mv));
    // SYS_STATUS emis a la cadence de publishBattery (BATTERY_TIMER) : tension seule.
    mavlink_msg_sys_status_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                                0, 0, 0,            // sensors present/enabled/health
                                0,                  // load
                                battMv_,            // voltage_battery (mV)
                                -1,                 // current_battery (non mesure ici)
                                -1,                 // battery_remaining (inconnu)
                                0, 0, 0, 0, 0, 0);
    sendMessage();
}

void ConnectorMavlink::publishJoint(joint_state_t* js) {
    // BAMBOO_ENCODERS : comptage cumulatif M1..M4 (tics).
    int32_t counts[4];
    for (int m = 0; m < 4; m++) counts[m] = (int32_t)lroundf(js[m].position);
    mavlink_msg_bamboo_encoders_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                                     (uint32_t)millis(), counts);
    sendMessage();
}

void ConnectorMavlink::publishMag(MAGInterface::Mag_t m) {
    // BAMBOO_MAG : mx/my/mz en uT. magnetic_field est en TESLA -> uT = T * 1e6.
    // Extension propre a l'ESP32 WaveShare (AK09918C) ; l'app calcule le cap boussole.
    mavlink_msg_bamboo_mag_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                                (uint32_t)millis(),
                                m.magnetic_field.x * 1.0e6f,
                                m.magnetic_field.y * 1.0e6f,
                                m.magnetic_field.z * 1.0e6f);
    sendMessage();
}
// Pas encodes dans ce dialecte (le HUD app n'expose pas range/wifi de l'ESP32).
void ConnectorMavlink::publishRange(Range::Range_t) {}
void ConnectorMavlink::publishWifi(DeviceWifi::DeviceWifi_t*) {}

// --- twist courant (reutilise les membres de base) ---
void ConnectorMavlink::setTwist(double linear_x, double linear_y, double angular_z) {
    twist_msg_.linear_x = linear_x;
    twist_msg_.linear_y = linear_y;
    twist_msg_.angular_z = angular_z;
}
float ConnectorMavlink::getTwistX() { return twist_msg_.linear_x; }
float ConnectorMavlink::getTwistY() { return twist_msg_.linear_y; }
float ConnectorMavlink::getTwistZ() { return twist_msg_.angular_z; }

// --- setters de cache (utilises par le connecteur Web, sans objet ici) ---
void ConnectorMavlink::setImu(float, float, float, float, float, float, float, float, float, float) {}
void ConnectorMavlink::setBattery(Battery::Battery_t) {}
void ConnectorMavlink::setMag(float, float, float) {}
void ConnectorMavlink::setOdometry(Odometry::Odometry_data) {}
void ConnectorMavlink::setRange(float) {}
void ConnectorMavlink::setRange(float, float, float) {}
void ConnectorMavlink::setJointStateList(joint_state_t*) {}

// --- temps (horloge ESP32 = millis()) ---
bool ConnectorMavlink::syncTime() { return true; }
int64_t ConnectorMavlink::getMillis() { return millis(); }
struct timespec ConnectorMavlink::getTime() {
    struct timespec tp = {0};
    unsigned long now = millis();
    tp.tv_sec  = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// ======================================================================= //
//  Parametres                                                             //
// ======================================================================= //
float ConnectorMavlink::paramGet(uint16_t idx) {
    if (idx <= P_MOT4_KD) {
        uint8_t k = idx % 3; // 0=Kp 1=Ki 2=Kd (partage entre les 4 moteurs)
        return (k == 0) ? Pid_.P : (k == 1 ? Pid_.I : Pid_.D);
    }
    switch (idx) {
    case P_YAW_KP: return yawP_;
    case P_YAW_KI: return yawI_;
    case P_YAW_KD: return yawD_;
    case P_WHEEL_CPR:  return WSM_CPR;
    case P_WHEEL_CIRC: return WSM_CIRC_MM;
    case P_WHEEL_APB:  return WSM_APB_MM;
    case P_CAR_TYPE:   return WSM_CAR_TYPE;
    default: return 0;
    }
}

int ConnectorMavlink::paramSetByName(const char* name, float value) {
    uint16_t idx;
    for (idx = 0; idx < PARAM_COUNT; idx++) {
        // param_id MAVLink : jusqu'a 16 octets, non termine si plein.
        if (strncmp(name, s_param_name[idx], 16) == 0) break;
    }
    if (idx >= PARAM_COUNT) return -1;

    if (idx <= P_MOT4_KD) {
        uint8_t k = idx % 3;
        float p = Pid_.P, i = Pid_.I, d = Pid_.D;
        if (k == 0) p = value; else if (k == 1) i = value; else d = value;
        Pid_.P = p; Pid_.I = i; Pid_.D = d;
        if (pidCallback_ != NULL) {
            Pid_t pid; pid.P = p; pid.I = i; pid.D = d;
            pidCallback_(&pid, "ConnectorMavlink"); // applique aux 4 moteurs
        }
        return idx;
    }
    switch (idx) {
    case P_YAW_KP: yawP_ = value; return idx;
    case P_YAW_KI: yawI_ = value; return idx;
    case P_YAW_KD: yawD_ = value; return idx;
    // Geometrie/CAR_TYPE : placeholder read-only (kinematics fige a la compilation).
    // On echo la valeur reelle pour ne pas mentir au GCS.
    case P_WHEEL_CPR: case P_WHEEL_CIRC: case P_WHEEL_APB: case P_CAR_TYPE:
        return idx;
    default: return -1;
    }
}

void ConnectorMavlink::sendParam(uint16_t idx) {
    if (idx >= PARAM_COUNT) return;
    mavlink_msg_param_value_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                                 s_param_name[idx], paramGet(idx),
                                 MAV_PARAM_TYPE_REAL32, PARAM_COUNT, idx);
    sendMessage();
}

// ======================================================================= //
//  Commandes                                                              //
// ======================================================================= //
void ConnectorMavlink::sendCommandAck(uint16_t command, uint8_t result) {
    mavlink_msg_command_ack_pack(MAV_SYS_ID_ESP32, MAV_COMP_ID, &s_tx_msg,
                                 command, result, 0, 0,
                                 0, 0 /* target_system/component : broadcast */);
    sendMessage();
}

void ConnectorMavlink::handleCommandLong() {
    const mavlink_message_t *msg = &s_rx_ready;
    uint16_t cmd = mavlink_msg_command_long_get_command(msg);
    float p1 = mavlink_msg_command_long_get_param1(msg);
    uint8_t result = MAV_RESULT_ACCEPTED;

    switch (cmd) {
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        sendCommandAck(cmd, MAV_RESULT_ACCEPTED); // ACK avant de partir
        if ((int)p1 == 1) ESP.restart();
        return;
    // Recalibrage du biais gyro (param1=1 = gyro, semantique MAVLink standard). On ne
    // recalibre pas dans ce handler (bloquant ~2 s) : on pose un drapeau consomme par la
    // boucle firmware, et on ACK ACCEPTED tout de suite (accepte, pas encore termine).
    // Robot suppose immobile. param1!=1 (accel/mag/...) : non gere -> UNSUPPORTED.
    case MAV_CMD_PREFLIGHT_CALIBRATION:
        if ((int)p1 == 1) { gyroCalReq_ = true; result = MAV_RESULT_ACCEPTED; }
        else              { result = MAV_RESULT_UNSUPPORTED; }
        break;
    // Fonctions absentes sur cette carte (servo ST3215 hors perimetre, pas de
    // persistance flash, pas de controleur yaw ni de reset odometrie cote
    // connecteur) : ACK honnete UNSUPPORTED.
    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_PREFLIGHT_STORAGE:
    case MAV_CMD_USER_1:
    case MAV_CMD_USER_2:
    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }
    sendCommandAck(cmd, result);
}

// Consommation du drapeau de recalibrage gyro (pose par handleCommandLong). La boucle
// firmware l'appelle a chaque tour ; retourne true UNE seule fois par demande.
bool ConnectorMavlink::takeGyroCalRequest() {
    if (!gyroCalReq_) return false;
    gyroCalReq_ = false;
    return true;
}

// ======================================================================= //
//  Reception : parsing + routage                                          //
// ======================================================================= //
void ConnectorMavlink::parseByte(uint8_t b) {
    if (mavlink_parse_char(MAVLINK_COMM_0, b, &s_rx_parse, &s_rx_status)) {
        // Trame complete + CRC OK : on route tout de suite (mono-thread).
        memcpy(&s_rx_ready, &s_rx_parse, sizeof(mavlink_message_t));
        s_rx_flag = true;
        routeMessage();
        s_rx_flag = false;
    }
}

void ConnectorMavlink::routeMessage() {
    mavlink_message_t *msg = &s_rx_ready;
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_BAMBOO_CMD_VEL: {
        // vx/wz en m/s, rad/s (SI). Passe par twistCallback -> setTwist +
        // prev_cmd_time = millis() (evite le fail-safe 500 ms) + toggle LED.
        const float vx = mavlink_msg_bamboo_cmd_vel_get_vx(msg);
        const float wz = mavlink_msg_bamboo_cmd_vel_get_wz(msg);
        if (twistCallback_ != NULL) {
            Twist_t tw; tw.linear_x = vx; tw.linear_y = 0; tw.angular_z = wz;
            twistCallback_(&tw, "ConnectorMavlink");
        } else {
            setTwist(vx, 0, wz);
        }
        break;
    }
    case MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM:
        // Pas de chemin connecteur -> moteur PWM open-loop (differe, cf. plan).
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        for (uint16_t i = 0; i < PARAM_COUNT; i++) sendParam(i);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        int16_t pidx = mavlink_msg_param_request_read_get_param_index(msg);
        if (pidx >= 0) {
            sendParam((uint16_t)pidx);
        } else {
            char name[17] = {0};
            mavlink_msg_param_request_read_get_param_id(msg, name);
            for (uint16_t i = 0; i < PARAM_COUNT; i++)
                if (strncmp(name, s_param_name[i], 16) == 0) { sendParam(i); break; }
        }
        break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
        char name[17] = {0};
        mavlink_msg_param_set_get_param_id(msg, name);
        float val = mavlink_msg_param_set_get_param_value(msg);
        int idx = paramSetByName(name, val);
        if (idx >= 0) sendParam((uint16_t)idx); // echo de la valeur appliquee
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG:
        handleCommandLong();
        break;
    default:
        break;
    }
}

#endif // ENABLE_MAVLINK
