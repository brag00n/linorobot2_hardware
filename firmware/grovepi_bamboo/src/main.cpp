/*
 * main.cpp - Firmware carte capteurs "GrovePi+ Bambou" (ATmega328P @ 16 MHz).
 *
 * Detourne l'ATmega328P de la GrovePi+ v3.0 en microcontrolleur autonome qui
 * agrege un IMU MPU6050 (I2C), 4 telemetres ultrason HC-SR04 et un telemetre IR
 * Sharp GP2Y0A710K0F, et les expose sur l'UART dans le MEME format de trame que
 * la STM32 Bamboo v4.
 *
 * Ce fichier ne fait plus que l'ORCHESTRATION : chaque device vit dans lib/
 * (Mpu6050, Ultrasonic, SharpIR), le protocole dans lib/Protocol, la config
 * persistante dans lib/DeviceConfig. Miroir des app_*.c / bsp_*.c de la STM32.
 *
 * --- Protocole (voir README.md et lib/Protocol/Protocol.h) ------------------
 *   Trame : [0xFF][ID][LEN][FUNC][donnees...][CHK]  (little-endian)
 *     ID 0xFC hote->carte / 0xFB carte->hote ; LEN = taille_totale-2 ;
 *     CHK = somme(octets[2..fin-1]) & 0xFF.
 *   0x50 REQUEST_DATA [subfunc][param] : lecture a la demande
 *   0x51 VERSION      [major][minor][patch]      (= test de contact)
 *   0x52 TIME_SYNC    req [seq] -> ack [seq][t_board u32]  (echo horloge, synchro)
 *   0x53 STATUS       [board_state][n]+n*[dev_id][health]
 *   0x54 CONFIG       [dev_id][enabled][pinA][pinB][period u16][health]
 *   0x55 SET_CONFIG   [dev_id][enabled][pinA][pinB][period u16]   (RAM seule)
 *   0x56 CONFIG_ACTION [action][guard]  -> ack [action][result]   (EEPROM)
 *   0x60/0x61/0x62 REPORT_* : [ts u32 ms] en tete + donnees (auto-report)
 *
 *   Horodatage : chaque trame report porte un timestamp (horloge monotone
 *   interne, Timer2 ISR). TIME_SYNC permet a l'hote d'estimer l'offset
 *   carte<->ROS pour fusionner les mesures de plusieurs cartes.
 *
 * --- Budget temps reel ------------------------------------------------------
 *   pulseIn() (HC-SR04) est BLOQUANT : on lit UN capteur par tour de boucle
 *   (round-robin, canaux desactives sautes) et on met a jour l'IMU a CHAQUE
 *   tour -> angle reactif, chaque ultrason rafraichi a ~loop/N.
 *
 * --- Cablage (defauts, reconfigurable a chaud par SET_CONFIG) ----------------
 *   MPU6050 : SDA=A4, SCL=A5, adr 0x68.  HC-SR04 : Trig=D2/D4/D6/D8,
 *   Echo=D3/D5/D7/D9.  Sharp IR : Vo=A0, VCC/GND doubles, cond. ~10 uF.
 *   NE PAS utiliser D0/D1 (UART) ni A4/A5 (I2C).
 */
#include <Arduino.h>
#include <Wire.h>
#include "Protocol.h"
#include "Clock.h"
#include "Mpu6050.h"
#include "Ultrasonic.h"
#include "SharpIR.h"
#include "DeviceConfig.h"

#ifndef FW_VERSION_MAJOR
#define FW_VERSION_MAJOR 0
#endif
#ifndef FW_VERSION_MINOR
#define FW_VERSION_MINOR 2
#endif
#ifndef FW_VERSION_PATCH
#define FW_VERSION_PATCH 0
#endif

static const uint32_t BAUD = 115200;

// --- Devices ----------------------------------------------------------------
static Mpu6050      imu;
static Ultrasonic   sonar[4];
static SharpIR      ir;
static DeviceConfig cfg;

// --- Etat ordonnanceur (timers par device) ---------------------------------
static uint8_t  sonarIdx = 0;
static uint32_t imuLastMs = 0, ultraLastMs = 0, irLastMs = 0;

// --- Emission des trames report ---------------------------------------------
static void sendVersion() {
  uint8_t d[3] = { FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH };
  sendFrame(FUNC_VERSION, d, 3);
}

// Les trames report portent un timestamp uint32 (ms, horloge monotone interne)
// en TETE de payload -> l'hote fusionne les mesures dans un referentiel commun.
static void sendImu() {
  uint8_t d[20];
  put32(&d[0], Clock::nowMs());
  put16(&d[4],  (uint16_t)imu.roll100());
  put16(&d[6],  (uint16_t)imu.pitch100());
  put16(&d[8],  (uint16_t)imu.ax()); put16(&d[10], (uint16_t)imu.ay());
  put16(&d[12], (uint16_t)imu.az()); put16(&d[14], (uint16_t)imu.gx());
  put16(&d[16], (uint16_t)imu.gy()); put16(&d[18], (uint16_t)imu.gz());
  sendFrame(FUNC_REPORT_IMU, d, 20);
}

static void sendUltra() {
  uint8_t d[12];
  put32(&d[0], Clock::nowMs());
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t v = cfg.slot(SLOT_U0 + i).enabled ? sonar[i].last() : DIST_NONE;
    put16(&d[4 + 2 * i], v);
  }
  sendFrame(FUNC_REPORT_ULTRA, d, 12);
}

static void sendIr() {
  uint8_t d[8];
  put32(&d[0], Clock::nowMs());
  put16(&d[4], ir.last());       // distance mm (0xFFFF = hors plage)
  put16(&d[6], ir.rawAdc());     // adc brut (pour calibration)
  sendFrame(FUNC_REPORT_IR, d, 8);
}

// --- Sante d'un slot (octet health de STATUS/CONFIG) ------------------------
static uint8_t slotHealth(uint8_t s) {
  if (!cfg.slot(s).enabled) return HLTH_DISABLED;
  switch (s) {
    case SLOT_IMU: return imu.health();
    case SLOT_IR:  return ir.health();
    case SLOT_U0: case SLOT_U1: case SLOT_U2: case SLOT_U3:
      return sonar[s - SLOT_U0].health();
    case SLOT_ULTRA: {                        // groupe : pire canal active
      uint8_t worst = HLTH_OK;
      for (uint8_t i = 0; i < 4; i++)
        if (cfg.slot(SLOT_U0 + i).enabled) {
          uint8_t h = sonar[i].health();
          if (h > worst) worst = h;
        }
      return worst;
    }
    default: return HLTH_OK;
  }
}

static void sendConfig(uint8_t devId) {
  int8_t s = DeviceConfig::slotOf(devId);
  if (s < 0) return;
  DevCfg &c = cfg.slot(s);
  uint8_t d[7];
  d[0] = devId; d[1] = c.enabled; d[2] = c.pinA; d[3] = c.pinB;
  put16(&d[4], c.periodMs);
  d[6] = slotHealth(s);
  sendFrame(FUNC_CONFIG, d, 7);
}

static void sendStatus() {
  uint8_t d[2 + NUM_CFG * 2];
  uint8_t k = 2, board = BOARD_OK;
  for (uint8_t s = 0; s < NUM_CFG; s++) {
    uint8_t h = slotHealth(s);
    d[k++] = DeviceConfig::devIdOf(s);
    d[k++] = h;
    if (cfg.slot(s).enabled && healthIsError(h)) {
      if (s == SLOT_IMU) board = BOARD_FAULT;               // capteur principal
      else if (board < BOARD_DEGRADED) board = BOARD_DEGRADED;
    }
  }
  d[0] = board;
  d[1] = NUM_CFG;
  sendFrame(FUNC_STATUS, d, k);
}

static void sendActionAck(uint8_t action, uint8_t result) {
  uint8_t d[2] = { action, result };
  sendFrame(FUNC_CONFIG_ACTION, d, 2);
}

// --- (Re)configuration materielle depuis la table de config -----------------
static void applySlot(uint8_t s) {
  DevCfg &c = cfg.slot(s);
  switch (s) {
    case SLOT_IMU: imu.begin(c.pinA); break;                // pinA = adresse I2C
    case SLOT_IR:  ir.begin(c.pinA);  break;                // pinA = broche analog
    case SLOT_U0: case SLOT_U1: case SLOT_U2: case SLOT_U3:
      sonar[s - SLOT_U0].setPins(c.pinA, c.pinB); break;
    default: break;                                         // groupe ULTRA : rien
  }
}

static void applyAllConfig() {
  for (uint8_t s = 0; s < NUM_CFG; s++) applySlot(s);
}

// --- Reception : commandes hote -> carte ------------------------------------
static void handleFrame(uint8_t func, const uint8_t *data, uint8_t n) {
  // TIME_SYNC : echo immediat de l'horloge monotone (l'horloge n'est PAS modifiee).
  // L'hote mesure t1/t2 autour de l'echange et estime l'offset carte<->ROS.
  if (func == FUNC_TIME_SYNC && n >= 1) {
    uint8_t d[5];
    d[0] = data[0];                    // seq echo (correlation requete/reponse)
    put32(&d[1], Clock::nowMs());
    sendFrame(FUNC_TIME_SYNC, d, 5);
    return;
  }

  if (func == FUNC_REQUEST_DATA && n >= 1) {
    uint8_t param = (n >= 2) ? data[1] : DEV_ALL;
    switch (data[0]) {
      case FUNC_VERSION:      sendVersion();     break;
      case FUNC_STATUS:       sendStatus();      break;
      case FUNC_CONFIG:       sendConfig(param); break;
      case FUNC_REPORT_IMU:   sendImu();         break;
      case FUNC_REPORT_ULTRA: sendUltra();       break;
      case FUNC_REPORT_IR:    ir.read(); sendIr(); break;
      default: break;
    }
    return;
  }

  // SET_CONFIG : applique en RAM (jamais d'EEPROM ici) + reconfig materielle.
  if (func == FUNC_SET_CONFIG && n >= 6) {
    uint8_t devId = data[0];
    int8_t s = DeviceConfig::slotOf(devId);
    if (s < 0) return;
    DevCfg &c = cfg.slot(s);
    if (data[1] != CFG_KEEP8) c.enabled = data[1] ? 1 : 0;
    if (data[2] != CFG_KEEP8) c.pinA = data[2];
    if (data[3] != CFG_KEEP8) c.pinB = data[3];
    uint16_t period = get16(&data[4]);
    if (period != 0) c.periodMs = period;                   // 0 = garder
    applySlot(s);
    sendConfig(devId);                                       // accuse par un 0x54
    return;
  }

  // CONFIG_ACTION : sauvegarde / defauts / reload EEPROM (trame dediee, gardee).
  if (func == FUNC_CONFIG_ACTION && n >= 2) {
    uint8_t action = data[0], guard = data[1];
    if (guard != SAVE_VERIFY) { sendActionAck(action, CFG_RES_BADGUARD); return; }
    uint8_t res;
    switch (action) {
      case CFG_ACT_SAVE:
        res = cfg.save();
        break;
      case CFG_ACT_DEFAULTS:
        cfg.loadDefaults(); applyAllConfig(); res = cfg.save();
        break;
      case CFG_ACT_RELOAD:
        res = cfg.reload(); if (res == CFG_RES_DONE) applyAllConfig();
        break;
      default:
        res = CFG_RES_BADGUARD;                             // action inconnue
        break;
    }
    sendActionAck(action, res);
    return;
  }
}

static FrameParser parser(handleFrame);

static void rxPoll() {
  while (Serial.available()) parser.feed((uint8_t)Serial.read());
}

// UN ultrason par appel (round-robin), canaux desactives sautes.
static void sonarStep() {
  for (uint8_t tries = 0; tries < 4; tries++) {
    uint8_t i = sonarIdx;
    sonarIdx = (sonarIdx + 1) & 3;
    if (cfg.slot(SLOT_U0 + i).enabled) { sonar[i].ping(); return; }
  }
}

static bool due(uint32_t &last, uint16_t period, uint32_t now) {
  if (period == 0) return false;                            // 0 = report desactive
  if (now - last < period) return false;
  last = now;
  return true;
}

// --- Arduino ----------------------------------------------------------------
void setup() {
  Serial.begin(BAUD);
  Clock::begin();       // horloge monotone (Timer2 ISR) : referentiel de temps
  Wire.begin();
  Wire.setClock(400000);
  cfg.begin();          // charge EEPROM si valide, sinon defauts compiles
  applyAllConfig();     // configure IMU / ultrasons / IR selon la table
  sendVersion();        // annonce au demarrage
}

void loop() {
  rxPoll();                                        // commandes eventuelles
  if (cfg.slot(SLOT_IMU).enabled) imu.update();    // IMU a CHAQUE tour (reactif)
  sonarStep();                                     // UN ultrason par tour

  uint32_t now = millis();
  if (cfg.slot(SLOT_IMU).enabled   && due(imuLastMs,   cfg.slot(SLOT_IMU).periodMs,   now)) sendImu();
  if (cfg.slot(SLOT_ULTRA).enabled && due(ultraLastMs, cfg.slot(SLOT_ULTRA).periodMs, now)) sendUltra();
  if (cfg.slot(SLOT_IR).enabled    && due(irLastMs,    cfg.slot(SLOT_IR).periodMs,    now)) { ir.read(); sendIr(); }
}
