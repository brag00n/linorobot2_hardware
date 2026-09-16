/*
 * main_mavlink.cpp - Orchestration de la carte capteurs GrovePi+ en MAVLink v2.
 *
 * Variante MAVLink de main.cpp (trames binaires) : MEME ordonnancement temps reel
 * (IMU a chaque tour, UN ultrason par tour en round-robin, timers due() par device)
 * mais l'I/O passe par le transport MavSensors (dialecte bamboo) au lieu des trames
 * maison. Selectionnee a la compilation par -D ENABLE_MAVLINK ; main.cpp est alors
 * neutralisee (#ifndef ENABLE_MAVLINK) et c'est ce fichier qui fournit setup()/loop().
 *
 * Telemetrie : ATTITUDE (MPU6050 roll/pitch), DISTANCE_SENSOR x5 (4 HC-SR04 id 0..3
 * + 1 Sharp IR id 4). Config : PARAM_* (enable/periode) + COMMAND_LONG PREFLIGHT_STORAGE
 * (EEPROM). Adressage sysid 4 / compid 1.
 */
#include <Arduino.h>

#ifdef ENABLE_MAVLINK

#include <Wire.h>
#include "Protocol.h"       // codes CFG_ACT_* / CFG_RES_* / SAVE_VERIFY / health
#include "Clock.h"
#include "Mpu6050.h"
#include "Ultrasonic.h"
#include "SharpIR.h"
#include "DeviceConfig.h"
#include "MavSensors.h"

static const uint32_t BAUD = 115200;

// --- Devices (identiques a main.cpp) ----------------------------------------
static Mpu6050      imu;
static Ultrasonic   sonar[4];
static SharpIR      ir;
static DeviceConfig cfg;
static MavSensors   mav;

// --- Etat ordonnanceur (timers par device) ---------------------------------
static uint8_t  sonarIdx = 0;
static uint32_t imuLastMs = 0, ultraLastMs = 0, irLastMs = 0;

// --- (Re)configuration materielle depuis la table de config -----------------
// Callback fourni a MavSensors (applique apres un PARAM_SET). Miroir de main.cpp.
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

// Callback PREFLIGHT_STORAGE : commit/defauts/reload EEPROM (ex CONFIG_ACTION 0x56).
static uint8_t storageAction(uint8_t action) {
    switch (action) {
    case CFG_ACT_SAVE:
        return cfg.save();
    case CFG_ACT_DEFAULTS: {
        cfg.loadDefaults(); applyAllConfig(); return cfg.save();
    }
    case CFG_ACT_RELOAD: {
        uint8_t res = cfg.reload();
        if (res == CFG_RES_DONE) applyAllConfig();
        return res;
    }
    default:
        return CFG_RES_BADGUARD;
    }
}

// --- Emission de la telemetrie (miroir des sendImu/sendUltra/sendIr) --------
static void sendImu() {
    mav.emitImu(imu.roll100(), imu.pitch100());
}

static void sendUltra() {
    // Un DISTANCE_SENSOR par canal actif (id 0..3). Canal coupe -> saute (pas de trame).
    for (uint8_t i = 0; i < 4; i++)
        if (cfg.slot(SLOT_U0 + i).enabled)
            mav.emitDistance(i, sonar[i].last(), false /* ultrason */);
}

static void sendIr() {
    mav.emitDistance(4, ir.last(), true /* infrarouge */);   // id 4 = Sharp IR
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
    if (period == 0) return false;                           // 0 = report desactive
    if (now - last < period) return false;
    last = now;
    return true;
}

// --- Arduino ----------------------------------------------------------------
void setup() {
    Serial.begin(BAUD);
    Clock::begin();       // horloge monotone (Timer2 ISR)
    Wire.begin();
    Wire.setClock(400000);
    cfg.begin();          // charge EEPROM si valide, sinon defauts compiles
    applyAllConfig();     // configure IMU / ultrasons / IR selon la table
    mav.begin(&cfg, applySlot, storageAction);
}

void loop() {
    mav.poll();                                      // RX MAVLink + HEARTBEAT 1 Hz
    if (cfg.slot(SLOT_IMU).enabled) imu.update();    // IMU a CHAQUE tour (reactif)
    sonarStep();                                     // UN ultrason par tour

    uint32_t now = millis();
    if (cfg.slot(SLOT_IMU).enabled   && due(imuLastMs,   cfg.slot(SLOT_IMU).periodMs,   now)) sendImu();
    if (cfg.slot(SLOT_ULTRA).enabled && due(ultraLastMs, cfg.slot(SLOT_ULTRA).periodMs, now)) sendUltra();
    if (cfg.slot(SLOT_IR).enabled    && due(irLastMs,    cfg.slot(SLOT_IR).periodMs,    now)) { ir.read(); sendIr(); }
}

#endif // ENABLE_MAVLINK
