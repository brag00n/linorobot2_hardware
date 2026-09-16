#include "ConnectorSerialFrame.hpp"
#include <math.h>

// Geometrie roue : reprise des macros de config (bamboov310_config.h) pour repondre
// aux requetes CAR_TYPE / WHEEL_GEOM. Valeurs PLACEHOLDER tant que la geometrie
// reelle de la Waveshare n'est pas mesuree (cf. caveat calibration du plan).
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

#define WSF_CPR      ((uint16_t)(COUNTS_PER_REV1))
#define WSF_CIRC_MM  (((float)WHEEL_DIAMETER) * PI * 1000.0f)      // circonference en mm
#define WSF_APB_MM   (((float)LR_WHEELS_DISTANCE) * 0.5f * 1000.0f) // demi-voie/empattement en mm

// Version firmware de ce connecteur (board-id ESP32 distinct pour les logs hote)
// 1.1.0 : orientation IMU (filtre complementaire) + emission magneto AK09918 (0x0B).
// 1.2.0 : transport MAVLink v2 (dialecte bamboo) selectionnable via ENABLE_MAVLINK.
#define WSF_VER_MAJOR 1
#define WSF_VER_MINOR 2
#define WSF_VER_PATCH 0

// --- funcodes (identiques a la STM32, cf. ros_monitor.py / RobotComSerial.py) ---
#define SF_REPORT_SPEED    0x0A // vitesse (Vx/Vy/Vz) + batterie
#define SF_REPORT_MAG      0x0B // champ magnetique AK09918 (mx/my/mz) -- extension ESP32
#define SF_REPORT_IMU_ATT  0x0C // attitude IMU (roll/pitch/yaw)
#define SF_REPORT_ENCODER  0x0D // comptage encodeurs M1..M4
#define SF_MOTOR           0x10 // pilotage moteur PWM (open loop)
#define SF_MOTION          0x12 // consigne cinematique (cmd_vel)
#define SF_SET_MOTOR_PID   0x13
#define SF_SET_YAW_PID     0x14
#define SF_CAR_TYPE        0x15
#define SF_WHEEL_GEOM      0x16
#define SF_REQUEST_DATA    0x50
#define SF_VERSION         0x51

// --- helpers little-endian ---
static inline void putU16(uint8_t* b, uint8_t& i, uint16_t v) { b[i++] = v & 0xFF; b[i++] = (v >> 8) & 0xFF; }
static inline void putI16(uint8_t* b, uint8_t& i, int16_t v)  { putU16(b, i, (uint16_t)v); }
static inline void putU32(uint8_t* b, uint8_t& i, uint32_t v) { b[i++] = v & 0xFF; b[i++] = (v >> 8) & 0xFF; b[i++] = (v >> 16) & 0xFF; b[i++] = (v >> 24) & 0xFF; }
static inline void putI32(uint8_t* b, uint8_t& i, int32_t v)  { putU32(b, i, (uint32_t)v); }
static inline int16_t getI16(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }
static inline uint16_t getU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline int16_t clampI16(long v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : (int16_t)v); }

// --- cycle de vie ---
bool ConnectorSerialFrame::initAgent(const connectorTimerCallbak_t ptimerCallback,
                                     connectorTwistCallbak_t ptwistCallback,
                                     connectorJointCallbak_t pJointCallback,
                                     connectorPidCallbak_t pPidCallback) {
    timerCallback_ = ptimerCallback;
    twistCallback_ = ptwistCallback;
    jointCallback_ = pJointCallback;
    pidCallback_   = pPidCallback;
    return true; // Serial deja demarre par setup() ; pas d'agent externe
}

bool ConnectorSerialFrame::isAvailable() { return true; }

// Toujours vrai en mode filaire (pas d'agent) : evite un fullStop() intempestif dans loop().
// Le fail-safe "pas de cmd_vel depuis 500 ms" de moveBase() reste le garde-fou actif.
bool ConnectorSerialFrame::pingAgent(int, int) { return true; }

bool ConnectorSerialFrame::listenAgent(long) {
    // 1) vider le flux entrant et dispatcher les trames hote -> carte (0xFF 0xFC ...)
    while (Serial.available() > 0) {
        parseByte((uint8_t)Serial.read());
    }
    // 2) tick de controle periodique (~10 Hz) : moveBase() + publishData() via le callback
    //    firmware, qui declenche l'auto-emission des trames de metriques.
    uint32_t now = millis();
    if (now - lastTick_ >= 100) {
        lastTick_ = now;
        if (timerCallback_ != NULL) {
            timer_t t; t.impl = NULL;
            timerCallback_(&t, 0, "ConnectorSerialFrame");
        }
    }
    return true;
}

// --- emission des metriques ---

void ConnectorSerialFrame::publishOdom(Odometry::Odometry_data odom) {
    // 0x0A : [ts u32][Vx i16 mm/s][Vy i16 mm/s][Vz i16 rad/s*1000][batt u8]
    uint8_t p[16]; uint8_t i = 0;
    putU32(p, i, (uint32_t)millis());
    putI16(p, i, clampI16(lroundf(odom.twist_twist_linear_x  * 1000.0f)));
    putI16(p, i, clampI16(lroundf(odom.twist_twist_linear_y  * 1000.0f)));
    putI16(p, i, clampI16(lroundf(odom.twist_twist_angular_z * 1000.0f)));
    p[i++] = battByte_; // derniere tension connue (rafraichie par publishBattery @0.5 Hz)
    sendFrame(SF_REPORT_SPEED, p, i);
}

void ConnectorSerialFrame::publishImu(IMUInterface::Imu_t m) {
    // Quaternion -> angles d'Euler (roll/pitch/yaw en rad), puis *10000 i16 comme la STM32.
    const float qx = m.orientation.x, qy = m.orientation.y, qz = m.orientation.z, qw = m.orientation.w;
    const float roll  = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));
    const float sinp  = 2.0f * (qw * qy - qz * qx);
    const float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(HALF_PI, sinp) : asinf(sinp);
    const float yaw   = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
    // 0x0C : [ts u32][roll i16][pitch i16][yaw i16]
    uint8_t p[10]; uint8_t i = 0;
    putU32(p, i, (uint32_t)millis());
    putI16(p, i, clampI16(lroundf(roll  * 10000.0f)));
    putI16(p, i, clampI16(lroundf(pitch * 10000.0f)));
    putI16(p, i, clampI16(lroundf(yaw   * 10000.0f)));
    sendFrame(SF_REPORT_IMU_ATT, p, i);
}

void ConnectorSerialFrame::publishBattery(Battery::Battery_t b) {
    long v = lroundf(b.voltage * 10.0f);
    battByte_ = (uint8_t)(v < 0 ? 0 : (v > 255 ? 255 : v)); // emise avec la trame 0x0A
}

void ConnectorSerialFrame::publishJoint(joint_state_t* js) {
    // 0x0D : [ts u32][M1 i32][M2 i32][M3 i32][M4 i32] (comptage cumulatif encodeurs)
    uint8_t p[20]; uint8_t i = 0;
    putU32(p, i, (uint32_t)millis());
    for (int m = 0; m < 4; m++) putI32(p, i, (int32_t)lroundf(js[m].position));
    sendFrame(SF_REPORT_ENCODER, p, i);
}

void ConnectorSerialFrame::publishMag(MAGInterface::Mag_t m) {
    // 0x0B : [ts u32][mx i16][my i16][mz i16] -- champ magnetique en 0.1 uT.
    // magnetic_field est en TESLA (AK09918 : brut * 0.15 uT = brut * 0.00000015 T) ;
    // T -> 0.1 uT = * 1e7. Champ terrestre ~5e-5 T -> ~500 counts (i16 large).
    // Extension propre a la carte ESP32 WaveShare (la STM32 n'emet jamais 0x0B) :
    // l'app calcule le cap boussole a partir de mx/my.
    uint8_t p[10]; uint8_t i = 0;
    putU32(p, i, (uint32_t)millis());
    putI16(p, i, clampI16(lroundf(m.magnetic_field.x * 1.0e7f)));
    putI16(p, i, clampI16(lroundf(m.magnetic_field.y * 1.0e7f)));
    putI16(p, i, clampI16(lroundf(m.magnetic_field.z * 1.0e7f)));
    sendFrame(SF_REPORT_MAG, p, i);
}
// Pas encodes dans ce protocole (le HUD app n'expose pas range/wifi de l'ESP32)
void ConnectorSerialFrame::publishRange(Range::Range_t) {}
void ConnectorSerialFrame::publishWifi(DeviceWifi::DeviceWifi_t*) {}

// --- twist courant (reutilise les membres de base) ---
void ConnectorSerialFrame::setTwist(double linear_x, double linear_y, double angular_z) {
    twist_msg_.linear_x = linear_x;
    twist_msg_.linear_y = linear_y;
    twist_msg_.angular_z = angular_z;
}
float ConnectorSerialFrame::getTwistX() { return twist_msg_.linear_x; }
float ConnectorSerialFrame::getTwistY() { return twist_msg_.linear_y; }
float ConnectorSerialFrame::getTwistZ() { return twist_msg_.angular_z; }

// --- setters de cache (utilises par le connecteur Web, sans objet ici) ---
void ConnectorSerialFrame::setImu(float, float, float, float, float, float, float, float, float, float) {}
void ConnectorSerialFrame::setBattery(Battery::Battery_t) {}
void ConnectorSerialFrame::setMag(float, float, float) {}
void ConnectorSerialFrame::setOdometry(Odometry::Odometry_data) {}
void ConnectorSerialFrame::setRange(float) {}
void ConnectorSerialFrame::setRange(float, float, float) {}
void ConnectorSerialFrame::setJointStateList(joint_state_t*) {}

// --- temps (horloge ESP32 = millis()) ---
bool ConnectorSerialFrame::syncTime() { return true; }
int64_t ConnectorSerialFrame::getMillis() { return millis(); }
struct timespec ConnectorSerialFrame::getTime() {
    struct timespec tp = {0};
    unsigned long now = millis();
    tp.tv_sec  = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

// --- construction/envoi d'une trame ---
void ConnectorSerialFrame::sendFrame(uint8_t func, const uint8_t* payload, uint8_t len) {
    uint8_t frame[5 + 64];
    uint8_t n = 0;
    frame[n++] = PTO_HEAD;
    frame[n++] = PTO_ID_TX;
    const uint8_t L = 3 + len;
    frame[n++] = L;
    frame[n++] = func;
    uint16_t sum = (uint16_t)L + func;
    for (uint8_t k = 0; k < len; k++) { frame[n++] = payload[k]; sum += payload[k]; }
    frame[n++] = (uint8_t)(sum & 0xFF);
    Serial.write(frame, n);
}

void ConnectorSerialFrame::sendVersion() {
    uint8_t p[3] = { WSF_VER_MAJOR, WSF_VER_MINOR, WSF_VER_PATCH };
    sendFrame(SF_VERSION, p, 3);
}

void ConnectorSerialFrame::replyRequest(uint8_t subfunc, uint8_t param) {
    switch (subfunc) {
        case SF_CAR_TYPE: { // 0x04 = FOURWHEEL cote hote (TB6612 2 canaux differentiel)
            uint8_t p[1] = { 0x04 };
            sendFrame(SF_CAR_TYPE, p, 1);
            break;
        }
        case SF_WHEEL_GEOM: { // [cpr u16][circ*10 u16][apb*10 u16]
            uint8_t p[6]; uint8_t i = 0;
            putU16(p, i, WSF_CPR);
            putU16(p, i, (uint16_t)lroundf(WSF_CIRC_MM * 10.0f));
            putU16(p, i, (uint16_t)lroundf(WSF_APB_MM * 10.0f));
            sendFrame(SF_WHEEL_GEOM, p, i);
            break;
        }
        case SF_SET_MOTOR_PID: { // [index u8][kp u16][ki u16][kd u16]
            uint8_t p[7]; uint8_t i = 0;
            p[i++] = param & 0x0F;
            putU16(p, i, (uint16_t)lroundf(Pid_.P * 1000.0f));
            putU16(p, i, (uint16_t)lroundf(Pid_.I * 1000.0f));
            putU16(p, i, (uint16_t)lroundf(Pid_.D * 1000.0f));
            sendFrame(SF_SET_MOTOR_PID, p, i);
            break;
        }
        case SF_SET_YAW_PID: { // echo du yaw memorise
            uint8_t p[7]; uint8_t i = 0;
            p[i++] = 0;
            putU16(p, i, (uint16_t)lroundf(yawP_ * 1000.0f));
            putU16(p, i, (uint16_t)lroundf(yawI_ * 1000.0f));
            putU16(p, i, (uint16_t)lroundf(yawD_ * 1000.0f));
            sendFrame(SF_SET_YAW_PID, p, i);
            break;
        }
        case SF_VERSION:
            sendVersion();
            break;
        default:
            break;
    }
}

// --- dispatch des trames hote -> carte ---
void ConnectorSerialFrame::dispatch(uint8_t func, const uint8_t* d, uint8_t plen) {
    switch (func) {
        case SF_MOTION: { // 0x12 [mode][Vx i16 mm/s][Vy i16][Vz i16 rad/s*1000]
            if (plen >= 7) {
                const float vx = getI16(d + 1) / 1000.0f;
                const float vz = getI16(d + 5) / 1000.0f;
                if (twistCallback_ != NULL) {
                    // Passe par le callback firmware -> setTwist + prev_cmd_time = millis()
                    // (evite le fail-safe 500 ms) + toggle LED.
                    Twist_t tw; tw.linear_x = vx; tw.linear_y = 0; tw.angular_z = vz;
                    twistCallback_(&tw, "ConnectorSerialFrame");
                } else {
                    setTwist(vx, 0, vz);
                }
            }
            break;
        }
        case SF_MOTOR: // 0x10 : pas de chemin connecteur -> moteur (differe, cf. plan)
            break;
        case SF_SET_MOTOR_PID: { // 0x13 [kp u16][ki u16][kd u16][args]
            if (plen >= 6) {
                const uint16_t kp = getU16(d), ki = getU16(d + 2), kd = getU16(d + 4);
                // Sentinelle de desactivation (6x 0xFF) : ne pas propager au PID.
                if (!(kp == 0xFFFF && ki == 0xFFFF && kd == 0xFFFF) && pidCallback_ != NULL) {
                    Pid_t pid; pid.P = kp / 1000.0f; pid.I = ki / 1000.0f; pid.D = kd / 1000.0f;
                    pidCallback_(&pid, "ConnectorSerialFrame");
                }
            }
            break;
        }
        case SF_SET_YAW_PID: { // 0x14 : pas de controleur yaw -> memorise pour echo
            if (plen >= 6) {
                yawP_ = getU16(d) / 1000.0f;
                yawI_ = getU16(d + 2) / 1000.0f;
                yawD_ = getU16(d + 4) / 1000.0f;
            }
            break;
        }
        case SF_REQUEST_DATA: // 0x50 [subfunc][param]
            if (plen >= 1) replyRequest(d[0], plen >= 2 ? d[1] : 0);
            break;
        case SF_VERSION: // 0x51
            sendVersion();
            break;
        default:
            break;
    }
}

// --- parseur d'entree (machine a etats, matche 0xFF 0xFC) ---
void ConnectorSerialFrame::parseByte(uint8_t b) {
    switch (pstate_) {
        case P_HEAD:
            if (b == PTO_HEAD) pstate_ = P_ID;
            break;
        case P_ID:
            if (b == PTO_ID_RX)      pstate_ = P_LEN;
            else if (b == PTO_HEAD)  pstate_ = P_ID;   // resync sur un nouvel entete
            else                     pstate_ = P_HEAD;
            break;
        case P_LEN:
            rxLen_ = b;
            // LEN >= 3 (FUNC + CHK au minimum) et (FUNC+payload+CHK) tient dans rxBuf_
            if (rxLen_ < 3 || (uint8_t)(rxLen_ - 1) > sizeof(rxBuf_)) { pstate_ = P_HEAD; break; }
            rxNeed_ = rxLen_ - 1; // FUNC + payload + CHK
            rxIdx_  = 0;
            pstate_ = P_DATA;
            break;
        case P_DATA:
            rxBuf_[rxIdx_++] = b;
            if (rxIdx_ >= rxNeed_) {
                const uint8_t func = rxBuf_[0];
                const uint8_t plen = rxLen_ - 3;
                const uint8_t chk  = rxBuf_[rxNeed_ - 1];
                uint16_t sum = rxLen_;
                for (uint8_t k = 0; k < (uint8_t)(rxNeed_ - 1); k++) sum += rxBuf_[k]; // FUNC + payload
                if ((uint8_t)(sum & 0xFF) == chk) {
                    dispatch(func, &rxBuf_[1], plen);
                }
                pstate_ = P_HEAD;
            }
            break;
    }
}
