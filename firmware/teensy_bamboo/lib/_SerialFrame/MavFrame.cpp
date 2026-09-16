#include "MavFrame.h"

// Garde de compilation : PlatformIO (LDF) compile TOUS les .cpp de lib/_SerialFrame/
// quel que soit le transport actif. Sans ENABLE_MAVLINK (ex. env trames binaires ou
// micro-ROS), le chemin d'inclusion du dialecte n'est pas fourni -> tout le corps est
// neutralise et cette unite de traduction devient vide. Meme motif que la STM32
// (mav_protocol.c) et l'ESP32 (ConnectorMavlink.cpp).
#ifdef ENABLE_MAVLINK

#include <math.h>

// Dialecte MAVLink v2 genere (header-only, vendore dans firmware/_common). On ne
// definit PAS MAVLINK_USE_CONVENIENCE_FUNCTIONS : on packe dans un mavlink_message_t
// puis on serialise a la main vers l'UART (Serial.write), comme les templates
// STM32/ESP32.
#include "bamboo/mavlink.h"

// Adressage MAVLink (cf. MavFrame.h / mav_protocol.h STM32 / ConnectorMavlink ESP32).
#define MAV_SYS_ID_TEENSY  (3)
#define MAV_COMP_ID        (1)   // MAV_COMP_ID_AUTOPILOT1

// ======================================================================= //
//  Etat TX/RX : un seul MavFrame -> etat de fichier. Le Teensy est         //
//  mono-thread (loop), aucun garde de concurrence necessaire.              //
// ======================================================================= //
static mavlink_message_t s_tx_msg;
static uint8_t           s_tx_buf[MAVLINK_MAX_PACKET_LEN];

static mavlink_message_t s_rx_parse;    // etat de parsing
static mavlink_status_t  s_rx_status;
static mavlink_message_t s_rx_ready;    // dernier message complet a router

// --- table de parametres (identique a la STM32 / ESP32 pour un codec hote uniforme) ---
// Le Teensy applique un jeu de gains PID unique aux 4 moteurs (onPid) : MOTn_Kx sont
// tous adosses au meme trio motP_/motI_/motD_ (comme l'echo unique de SerialFrame).
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
void MavFrame::begin(TickCb tick, TwistCb twist, PidCb pid) {
    tickCb_  = tick;
    twistCb_ = twist;
    pidCb_   = pid;
    // Serial deja demarre par setup() ; pas d'agent externe a joindre.
}

void MavFrame::poll() {
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
        if (tickCb_ != NULL) tickCb_();
    }
}

// --- geometrie / type de chassis ---
void MavFrame::setWheelGeom(uint16_t cpr, float circ_mm, float apb_mm) {
    cpr_    = cpr;
    circMm_ = circ_mm;
    apbMm_  = apb_mm;
}
void MavFrame::setCarType(uint8_t t) { carType_ = (float)t; }
void MavFrame::setCalibrate(CalibCb cb) { calibCb_ = cb; }

// --- emission de bas niveau ---
void MavFrame::sendMessage() {
    uint16_t len = mavlink_msg_to_send_buffer(s_tx_buf, &s_tx_msg);
    Serial.write(s_tx_buf, len);
}

void MavFrame::sendHeartbeat() {
    mavlink_msg_heartbeat_pack(MAV_SYS_ID_TEENSY, MAV_COMP_ID, &s_tx_msg,
                               MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
                               MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0, MAV_STATE_ACTIVE);
    sendMessage();
}

// --- emission des metriques (API SerialFrame) ---
void MavFrame::emitSpeed(float vx, float vy, float wz, uint8_t battByte) {
    // BAMBOO_WHEEL_STATE : vx/vy en m/s, wz en rad/s (unites SI, pas de mise a l'echelle).
    mavlink_msg_bamboo_wheel_state_pack(MAV_SYS_ID_TEENSY, MAV_COMP_ID, &s_tx_msg,
                                        (uint32_t)millis(), vx, vy, wz);
    sendMessage();
    // SYS_STATUS : tension batterie. Pas de mesure batterie sur ce banc (battByte=0) ;
    // l'octet Yahboom est en decivolts -> mV si un jour cable (byte * 100).
    battMv_ = (uint16_t)battByte * 100;
    mavlink_msg_sys_status_pack(MAV_SYS_ID_TEENSY, MAV_COMP_ID, &s_tx_msg,
                                0, 0, 0,            // sensors present/enabled/health
                                0,                  // load
                                battMv_,            // voltage_battery (mV)
                                -1,                 // current_battery (non mesure ici)
                                -1,                 // battery_remaining (inconnu)
                                0, 0, 0, 0, 0, 0);
    sendMessage();
}

void MavFrame::emitImu(float roll, float pitch, float yaw) {
    // ATTITUDE : roll/pitch/yaw en rad. Les vitesses angulaires ne sont pas exposees par
    // l'API emitImu (filtre complementaire) -> rates a 0 (yaw RELATIF, derive : caveat MPU6050).
    mavlink_msg_attitude_pack(MAV_SYS_ID_TEENSY, MAV_COMP_ID, &s_tx_msg,
                              (uint32_t)millis(), roll, pitch, yaw, 0.0f, 0.0f, 0.0f);
    sendMessage();
}

void MavFrame::emitEncoders(const int32_t m[4]) {
    // BAMBOO_ENCODERS : comptage cumulatif M1..M4 (tics, encodeurs quadrature reels).
    int32_t counts[4] = { m[0], m[1], m[2], m[3] };
    mavlink_msg_bamboo_encoders_pack(MAV_SYS_ID_TEENSY, MAV_COMP_ID, &s_tx_msg,
                                     (uint32_t)millis(), counts);
    sendMessage();
}

// ======================================================================= //
//  Parametres                                                             //
// ======================================================================= //
float MavFrame::paramGet(uint16_t idx) {
    if (idx <= P_MOT4_KD) {
        uint8_t k = idx % 3; // 0=Kp 1=Ki 2=Kd (partage entre les 4 moteurs)
        return (k == 0) ? motP_ : (k == 1 ? motI_ : motD_);
    }
    switch (idx) {
    case P_YAW_KP: return yawP_;
    case P_YAW_KI: return yawI_;
    case P_YAW_KD: return yawD_;
    case P_WHEEL_CPR:  return (float)cpr_;
    case P_WHEEL_CIRC: return circMm_;
    case P_WHEEL_APB:  return apbMm_;
    case P_CAR_TYPE:   return carType_;
    default: return 0;
    }
}

int MavFrame::paramSetByName(const char* name, float value) {
    uint16_t idx;
    for (idx = 0; idx < PARAM_COUNT; idx++) {
        // param_id MAVLink : jusqu'a 16 octets, non termine si plein.
        if (strncmp(name, s_param_name[idx], 16) == 0) break;
    }
    if (idx >= PARAM_COUNT) return -1;

    if (idx <= P_MOT4_KD) {
        uint8_t k = idx % 3;
        if (k == 0) motP_ = value; else if (k == 1) motI_ = value; else motD_ = value;
        if (pidCb_ != NULL) pidCb_(motP_, motI_, motD_); // applique aux 4 moteurs
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

void MavFrame::sendParam(uint16_t idx) {
    if (idx >= PARAM_COUNT) return;
    mavlink_msg_param_value_pack(MAV_SYS_ID_TEENSY, MAV_COMP_ID, &s_tx_msg,
                                 s_param_name[idx], paramGet(idx),
                                 MAV_PARAM_TYPE_REAL32, PARAM_COUNT, idx);
    sendMessage();
}

// ======================================================================= //
//  Commandes                                                              //
// ======================================================================= //
void MavFrame::sendCommandAck(uint16_t command, uint8_t result) {
    mavlink_msg_command_ack_pack(MAV_SYS_ID_TEENSY, MAV_COMP_ID, &s_tx_msg,
                                 command, result, 0, 0,
                                 0, 0 /* target_system/component : broadcast */);
    sendMessage();
}

void MavFrame::handleCommandLong() {
    const mavlink_message_t *msg = &s_rx_ready;
    uint16_t cmd = mavlink_msg_command_long_get_command(msg);
    float p1 = mavlink_msg_command_long_get_param1(msg);
    uint8_t result = MAV_RESULT_UNSUPPORTED;

    switch (cmd) {
    case MAV_CMD_PREFLIGHT_CALIBRATION:
        // Le besoin declencheur : recalibrer le biais gyro a la demande (param1=1,
        // convention MAVLink pour la calib gyro). L'ancien funcode 0x17 n'avait jamais
        // ete cable. calibrateGyro() ne reussit que carte immobile (garde-fou ImuAtt).
        if ((int)p1 == 1 && calibCb_ != NULL) {
            result = calibCb_() ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        } else {
            result = MAV_RESULT_DENIED; // calib non demandee (p1!=1) ou non cablee
        }
        break;
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        // Pas de bootloader logiciel expose cote Teensy (reflash par bouton). ACK honnete.
        result = MAV_RESULT_UNSUPPORTED;
        break;
    // Fonctions absentes sur cette carte (servos differes, pas de persistance flash,
    // pas de controleur yaw ni de reset odometrie cote transport) : ACK UNSUPPORTED.
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

// ======================================================================= //
//  Reception : parsing + routage                                          //
// ======================================================================= //
void MavFrame::parseByte(uint8_t b) {
    if (mavlink_parse_char(MAVLINK_COMM_0, b, &s_rx_parse, &s_rx_status)) {
        // Trame complete + CRC OK : on route tout de suite (mono-thread).
        memcpy(&s_rx_ready, &s_rx_parse, sizeof(mavlink_message_t));
        routeMessage();
    }
}

void MavFrame::routeMessage() {
    mavlink_message_t *msg = &s_rx_ready;
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_BAMBOO_CMD_VEL: {
        // vx/wz en m/s, rad/s (SI). Passe par twistCb -> shim twist + prev_cmd_time.
        const float vx = mavlink_msg_bamboo_cmd_vel_get_vx(msg);
        const float wz = mavlink_msg_bamboo_cmd_vel_get_wz(msg);
        if (twistCb_ != NULL) twistCb_(vx, 0.0f, wz);
        break;
    }
    case MAVLINK_MSG_ID_BAMBOO_MOTOR_PWM:
        // Pas de chemin PWM open-loop cote Teensy (differe, cf. plan).
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
