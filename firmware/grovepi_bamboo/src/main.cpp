/*
 * main.cpp - Firmware carte capteurs "GrovePi+ Bambou" (ATmega328P @ 16 MHz).
 *
 * Detourne l'ATmega328P de la GrovePi+ v3.0 en microcontrolleur autonome qui
 * agrege un IMU MPU6050 (I2C) et 4 telemetres ultrason HC-SR04, et les expose
 * sur l'UART dans le MEME format de trame que la STM32 Bamboo v4.
 *
 * --- Protocole (miroir de ../stm32_bamboo/tools/ros_monitor.py) -------------
 *   Trame little-endian :  [0xFF][ID][LEN][FUNC][donnees...][CHK]
 *     0xFF  = entete
 *     ID    = 0xFC hote->carte, 0xFB carte->hote
 *     LEN   = nb d'octets a partir de LEN inclus (= taille_totale - 2)
 *     FUNC  = code fonction (voir ci-dessous)
 *     CHK   = somme(octets[2..fin-1]) & 0xFF
 *
 *   FUNC :
 *     0x50 REQUEST_DATA (hote->carte) : payload [subfunc] -> renvoie la trame subfunc
 *     0x51 VERSION      (carte->hote) : [major, minor]
 *     0x60 REPORT_IMU   (carte->hote) : roll,pitch (int16 deg*100 FUSIONNES)
 *                                       + ax,ay,az,gx,gy,gz (int16 bruts)
 *     0x61 REPORT_ULTRA (carte->hote) : dist[4] (uint16 mm ; 0xFFFF = pas d'echo)
 *
 *   Auto-report : IMU et ultrasons sont diffuses spontanement (~20 Hz), comme
 *   l'auto-report de la STM32. REQUEST_DATA permet en plus une lecture a la demande.
 *
 * --- Budget temps reel (cf. discussion) -------------------------------------
 *   pulseIn() est BLOQUANT. Lire les 4 HC-SR04 d'affilee (pire cas ~4x le
 *   timeout) figerait le filtre complementaire. On lit donc UN capteur par tour
 *   de boucle (round-robin) et on met a jour l'IMU a CHAQUE tour -> angle
 *   reactif (~kHz possible), chaque ultrason rafraichi a ~loop/4.
 *
 * --- Cablage (a confirmer sur la carte reelle) ------------------------------
 *   MPU6050  : SDA=A4, SCL=A5 (bus I2C du 328P = ports I2C Grove). Adr 0x68.
 *   HC-SR04  : 1 capteur par port digital Grove (2 lignes signal = Trig+Echo).
 *              Defaut : Trig=D2/D4/D6/D8, Echo=D3/D5/D7/D9. 5 V direct (328P @5V).
 *   NE PAS utiliser D0/D1 (UART) ni A4/A5 (I2C).
 */
#include <Arduino.h>
#include <Wire.h>

// --- Protocole --------------------------------------------------------------
static const uint8_t PTO_HEAD  = 0xFF;
static const uint8_t PTO_ID_RX = 0xFC;   // hote -> carte
static const uint8_t PTO_ID_TX = 0xFB;   // carte -> hote
static const uint8_t FUNC_REQUEST_DATA = 0x50;
static const uint8_t FUNC_VERSION      = 0x51;
static const uint8_t FUNC_REPORT_IMU   = 0x60;
static const uint8_t FUNC_REPORT_ULTRA = 0x61;

#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR 0
#endif
#ifndef FW_VERSION_MINOR
#define FW_VERSION_MINOR 1
#endif

static const uint32_t BAUD = 115200;
static const uint16_t REPORT_PERIOD_MS = 50;   // auto-report ~20 Hz

// --- MPU6050 ----------------------------------------------------------------
static const uint8_t MPU_ADDR       = 0x68;
static const uint8_t MPU_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_ACCEL_XOUT = 0x3B;
static const float   ACC_LSB_PER_G   = 16384.0f;  // +-2 g
static const float   GYR_LSB_PER_DPS = 131.0f;    // +-250 deg/s
static const float   COMP_ALPHA      = 0.98f;     // poids gyro du filtre complementaire

// --- HC-SR04 (round-robin) --------------------------------------------------
static const uint8_t N_SONAR = 4;
static const uint8_t TRIG_PIN[N_SONAR] = {2, 4, 6, 8};
static const uint8_t ECHO_PIN[N_SONAR] = {3, 5, 7, 9};
static const uint16_t MAX_RANGE_CM   = 250;                       // portee plafonnee
static const uint32_t ECHO_TIMEOUT_US = (uint32_t)MAX_RANGE_CM * 58UL + 400UL;
static const uint16_t DIST_NONE = 0xFFFF;                          // pas d'echo

// Etat IMU fusionne
static float rollDeg = 0.0f, pitchDeg = 0.0f;
static int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
static uint32_t lastImuUs = 0;

// Distances courantes (mm), rafraichies une a la fois
static uint16_t distMm[N_SONAR] = {DIST_NONE, DIST_NONE, DIST_NONE, DIST_NONE};
static uint8_t  sonarIdx = 0;

static uint32_t lastReportMs = 0;

// --- Emission de trame -------------------------------------------------------
static void sendFrame(uint8_t func, const uint8_t *data, uint8_t n) {
  uint8_t len = n + 3;                       // LEN + FUNC + data + CHK, -1
  uint8_t sum = len + func;
  for (uint8_t i = 0; i < n; i++) sum += data[i];
  Serial.write(PTO_HEAD);
  Serial.write(PTO_ID_TX);
  Serial.write(len);
  Serial.write(func);
  if (n) Serial.write(data, n);
  Serial.write((uint8_t)(sum & 0xFF));
}

static inline void put16(uint8_t *b, int16_t v) {   // little-endian
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void sendVersion() {
  uint8_t d[2] = {FW_VERSION_MAJOR, FW_VERSION_MINOR};
  sendFrame(FUNC_VERSION, d, 2);
}

static void sendImu() {
  uint8_t d[16];
  put16(&d[0],  (int16_t)(rollDeg  * 100.0f));
  put16(&d[2],  (int16_t)(pitchDeg * 100.0f));
  put16(&d[4],  ax); put16(&d[6],  ay); put16(&d[8],  az);
  put16(&d[10], gx); put16(&d[12], gy); put16(&d[14], gz);
  sendFrame(FUNC_REPORT_IMU, d, 16);
}

static void sendUltra() {
  uint8_t d[N_SONAR * 2];
  for (uint8_t i = 0; i < N_SONAR; i++) {
    d[2 * i]     = (uint8_t)(distMm[i] & 0xFF);
    d[2 * i + 1] = (uint8_t)((distMm[i] >> 8) & 0xFF);
  }
  sendFrame(FUNC_REPORT_ULTRA, d, sizeof(d));
}

// --- MPU6050 ----------------------------------------------------------------
static bool mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00);                          // reveil (sort du sleep)
  return Wire.endTransmission() == 0;
}

static void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();                  // temperature (ignoree)
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

// Filtre complementaire : fusionne l'angle accel (absolu, bruite) et
// l'integration gyro (lisse, derive) a chaque tour de boucle.
static void imuUpdate() {
  mpuRead();
  uint32_t now = micros();
  float dt = (now - lastImuUs) * 1e-6f;
  lastImuUs = now;
  if (dt <= 0.0f || dt > 0.5f) return;       // premier tour / trou -> on saute

  float axf = ax / ACC_LSB_PER_G;
  float ayf = ay / ACC_LSB_PER_G;
  float azf = az / ACC_LSB_PER_G;
  float rollAcc  = atan2(ayf, azf) * 57.29578f;
  float pitchAcc = atan2(-axf, sqrt(ayf * ayf + azf * azf)) * 57.29578f;

  float gxDps = gx / GYR_LSB_PER_DPS;
  float gyDps = gy / GYR_LSB_PER_DPS;
  rollDeg  = COMP_ALPHA * (rollDeg  + gxDps * dt) + (1.0f - COMP_ALPHA) * rollAcc;
  pitchDeg = COMP_ALPHA * (pitchDeg + gyDps * dt) + (1.0f - COMP_ALPHA) * pitchAcc;
}

// --- HC-SR04 : un capteur par appel (round-robin) ---------------------------
static void sonarStep() {
  uint8_t i = sonarIdx;
  digitalWrite(TRIG_PIN[i], LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN[i], LOW);
  uint32_t dur = pulseIn(ECHO_PIN[i], HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) {
    distMm[i] = DIST_NONE;                    // pas d'echo (hors portee)
  } else {
    uint32_t mm = (dur * 10UL) / 58UL;        // aller-retour : 58 us/cm
    distMm[i] = (mm > 65534UL) ? 65534 : (uint16_t)mm;
  }
  sonarIdx = (sonarIdx + 1) % N_SONAR;
}

// --- Reception : commandes hote -> carte ------------------------------------
static uint8_t rxBuf[16];
static uint8_t rxLen = 0;

static void handleFrame(uint8_t func, const uint8_t *data, uint8_t n) {
  if (func == FUNC_REQUEST_DATA && n >= 1) {
    switch (data[0]) {
      case FUNC_VERSION:      sendVersion(); break;
      case FUNC_REPORT_IMU:   sendImu();     break;
      case FUNC_REPORT_ULTRA: sendUltra();   break;
      default: break;
    }
  }
}

// Machine a etats de reception (tolere le bruit, resync sur l'entete)
static void rxPoll() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (rxLen == 0) { if (b == PTO_HEAD) rxBuf[rxLen++] = b; continue; }
    if (rxLen == 1) { if (b == PTO_ID_RX) rxBuf[rxLen++] = b; else rxLen = 0; continue; }
    rxBuf[rxLen++] = b;
    if (rxLen >= 3) {
      uint8_t total = rxBuf[2] + 2;
      if (total < 5 || total > sizeof(rxBuf)) { rxLen = 0; continue; }
      if (rxLen == total) {
        uint8_t sum = 0;
        for (uint8_t i = 2; i < total - 1; i++) sum += rxBuf[i];
        if (sum == rxBuf[total - 1])
          handleFrame(rxBuf[3], &rxBuf[4], total - 5);
        rxLen = 0;
      }
    }
  }
}

// --- Arduino ----------------------------------------------------------------
void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);
  mpuInit();
  for (uint8_t i = 0; i < N_SONAR; i++) {
    pinMode(TRIG_PIN[i], OUTPUT);
    digitalWrite(TRIG_PIN[i], LOW);
    pinMode(ECHO_PIN[i], INPUT);
  }
  lastImuUs = micros();
  sendVersion();                              // annonce au demarrage
}

void loop() {
  rxPoll();                                   // commandes eventuelles
  imuUpdate();                                // IMU a CHAQUE tour (reactif)
  sonarStep();                                // UN ultrason par tour (round-robin)

  uint32_t now = millis();
  if (now - lastReportMs >= REPORT_PERIOD_MS) {
    lastReportMs = now;
    sendImu();
    sendUltra();
  }
}
