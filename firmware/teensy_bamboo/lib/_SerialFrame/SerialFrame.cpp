#include "SerialFrame.h"
#include <math.h>

// Version firmware de ce connecteur Teensy (board-id distinct pour les logs hote).
// 1.0.0 : premiere version trames binaires Teensy (0x0A vitesse+batt / 0x0C IMU /
// 0x0D encodeurs reels). Pas de magneto (MPU6050) -> pas de 0x0B. Servos differes.
// 1.1.0 : calibration du biais gyro au begin() (ImuAtt) -> derive yaw fortement reduite.
#define TSF_VER_MAJOR 1
#define TSF_VER_MINOR 1
#define TSF_VER_PATCH 0

// --- funcodes (identiques a la STM32 / ESP32, cf. RobotComSerial.py) ---
#define SF_REPORT_SPEED    0x0A // vitesse (Vx/Vy/Vz) + batterie
#define SF_REPORT_IMU_ATT  0x0C // attitude IMU (roll/pitch/yaw)
#define SF_REPORT_ENCODER  0x0D // comptage encodeurs M1..M4
#define SF_MOTOR           0x10 // pilotage moteur PWM (open loop) -- differe
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
void SerialFrame::begin(TickCb tick, TwistCb twist, PidCb pid) {
    tickCb_  = tick;
    twistCb_ = twist;
    pidCb_   = pid;
    // Serial deja demarre par setup() ; pas d'agent externe a joindre.
}

void SerialFrame::poll() {
    // 1) vider le flux entrant et dispatcher les trames hote -> carte (0xFF 0xFC ...)
    while (Serial.available() > 0) {
        parseByte((uint8_t)Serial.read());
    }
    // 2) tick de controle periodique (~10 Hz) : moveBase() + publishData() via le
    //    callback firmware, qui declenche l'auto-emission des trames de metriques.
    uint32_t now = millis();
    if (now - lastTick_ >= 100) {
        lastTick_ = now;
        if (tickCb_ != NULL) tickCb_();
    }
}

// --- geometrie / type de chassis ---
void SerialFrame::setWheelGeom(uint16_t cpr, float circ_mm, float apb_mm) {
    cpr_      = cpr;
    circMm10_ = (uint16_t)lroundf(circ_mm * 10.0f);
    apbMm10_  = (uint16_t)lroundf(apb_mm * 10.0f);
}
void SerialFrame::setCarType(uint8_t t) { carType_ = t; }

// --- emission des metriques ---
void SerialFrame::emitSpeed(float vx, float vy, float wz, uint8_t battByte) {
    // 0x0A : [ts u32][Vx i16 mm/s][Vy i16 mm/s][Vz i16 rad/s*1000][batt u8]
    uint8_t p[16]; uint8_t i = 0;
    putU32(p, i, (uint32_t)millis());
    putI16(p, i, clampI16(lroundf(vx * 1000.0f)));
    putI16(p, i, clampI16(lroundf(vy * 1000.0f)));
    putI16(p, i, clampI16(lroundf(wz * 1000.0f)));
    p[i++] = battByte;
    sendFrame(SF_REPORT_SPEED, p, i);
}

void SerialFrame::emitImu(float roll, float pitch, float yaw) {
    // 0x0C : [ts u32][roll i16][pitch i16][yaw i16] (rad * 10000, comme la STM32)
    uint8_t p[10]; uint8_t i = 0;
    putU32(p, i, (uint32_t)millis());
    putI16(p, i, clampI16(lroundf(roll  * 10000.0f)));
    putI16(p, i, clampI16(lroundf(pitch * 10000.0f)));
    putI16(p, i, clampI16(lroundf(yaw   * 10000.0f)));
    sendFrame(SF_REPORT_IMU_ATT, p, i);
}

void SerialFrame::emitEncoders(const int32_t m[4]) {
    // 0x0D : [ts u32][M1 i32][M2 i32][M3 i32][M4 i32] (comptage cumulatif, encodeurs reels)
    uint8_t p[20]; uint8_t i = 0;
    putU32(p, i, (uint32_t)millis());
    for (int k = 0; k < 4; k++) putI32(p, i, m[k]);
    sendFrame(SF_REPORT_ENCODER, p, i);
}

// --- construction/envoi d'une trame ---
void SerialFrame::sendFrame(uint8_t func, const uint8_t* payload, uint8_t len) {
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

void SerialFrame::sendVersion() {
    uint8_t p[3] = { TSF_VER_MAJOR, TSF_VER_MINOR, TSF_VER_PATCH };
    sendFrame(SF_VERSION, p, 3);
}

void SerialFrame::replyRequest(uint8_t subfunc, uint8_t param) {
    switch (subfunc) {
        case SF_CAR_TYPE: { // type de chassis cote hote (0x04 = FOURWHEEL)
            uint8_t p[1] = { carType_ };
            sendFrame(SF_CAR_TYPE, p, 1);
            break;
        }
        case SF_WHEEL_GEOM: { // [cpr u16][circ*10 u16][apb*10 u16]
            uint8_t p[6]; uint8_t i = 0;
            putU16(p, i, cpr_);
            putU16(p, i, circMm10_);
            putU16(p, i, apbMm10_);
            sendFrame(SF_WHEEL_GEOM, p, i);
            break;
        }
        case SF_SET_MOTOR_PID: { // [index u8][kp u16][ki u16][kd u16]
            uint8_t p[7]; uint8_t i = 0;
            p[i++] = param & 0x0F;
            putU16(p, i, (uint16_t)lroundf(motP_ * 1000.0f));
            putU16(p, i, (uint16_t)lroundf(motI_ * 1000.0f));
            putU16(p, i, (uint16_t)lroundf(motD_ * 1000.0f));
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
void SerialFrame::dispatch(uint8_t func, const uint8_t* d, uint8_t plen) {
    switch (func) {
        case SF_MOTION: { // 0x12 [mode][Vx i16 mm/s][Vy i16][Vz i16 rad/s*1000]
            if (plen >= 7) {
                const float vx = getI16(d + 1) / 1000.0f;
                const float vz = getI16(d + 5) / 1000.0f;
                if (twistCb_ != NULL) twistCb_(vx, 0.0f, vz);
            }
            break;
        }
        case SF_MOTOR: // 0x10 : pas de chemin connecteur -> moteur (differe, cf. plan)
            break;
        case SF_SET_MOTOR_PID: { // 0x13 [kp u16][ki u16][kd u16][args]
            if (plen >= 6) {
                const uint16_t kp = getU16(d), ki = getU16(d + 2), kd = getU16(d + 4);
                // Sentinelle de desactivation (6x 0xFF) : ne pas propager au PID.
                if (!(kp == 0xFFFF && ki == 0xFFFF && kd == 0xFFFF)) {
                    motP_ = kp / 1000.0f; motI_ = ki / 1000.0f; motD_ = kd / 1000.0f;
                    if (pidCb_ != NULL) pidCb_(motP_, motI_, motD_);
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
void SerialFrame::parseByte(uint8_t b) {
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
