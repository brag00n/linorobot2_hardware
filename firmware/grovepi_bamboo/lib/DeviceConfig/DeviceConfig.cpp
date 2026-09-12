#include "DeviceConfig.h"
#include <EEPROM.h>

// --- Format EEPROM ----------------------------------------------------------
static const uint16_t EE_MAGIC   = 0xB4C0;   // "Bamboo Config"
static const uint8_t  EE_LAYOUT  = 1;        // version de layout (incr. si struct change)
static const uint16_t EE_BASE    = 0;        // offset de depart en EEPROM
static const uint8_t  CFG_BYTES  = 5;        // octets serialises par DevCfg
// Taille totale : magic(2) + ver(1) + NUM_CFG*5 + crc(1).
static const uint8_t  EE_SIZE    = 2 + 1 + NUM_CFG * CFG_BYTES + 1;

// Intervalle minimal entre deux ecritures physiques (anti-rafale EEPROM).
static const uint32_t MIN_SAVE_INTERVAL_MS = 2000;

// --- Defauts compiles -------------------------------------------------------
static void fillDefaults(DevCfg *c) {
  c[SLOT_IMU]   = { 1, 0x68, CFG_KEEP8, 50 };   // adresse I2C dans pinA
  c[SLOT_ULTRA] = { 1, CFG_KEEP8, CFG_KEEP8, 50 };
  c[SLOT_U0]    = { 1, 2, 3, 0 };               // Trig/Echo D2/D3
  c[SLOT_U1]    = { 1, 4, 5, 0 };
  c[SLOT_U2]    = { 1, 6, 7, 0 };
  c[SLOT_U3]    = { 1, 8, 9, 0 };
  c[SLOT_IR]    = { 1, A0, CFG_KEEP8, 50 };
}

// --- Mapping dev_id <-> slot ------------------------------------------------
int8_t DeviceConfig::slotOf(uint8_t devId) {
  switch (devId) {
    case DEV_IMU:    return SLOT_IMU;
    case DEV_ULTRA:  return SLOT_ULTRA;
    case DEV_ULTRA0: return SLOT_U0;
    case DEV_ULTRA1: return SLOT_U1;
    case DEV_ULTRA2: return SLOT_U2;
    case DEV_ULTRA3: return SLOT_U3;
    case DEV_IR:     return SLOT_IR;
    default:         return -1;
  }
}

uint8_t DeviceConfig::devIdOf(uint8_t slot) {
  static const uint8_t ids[NUM_CFG] = {
    DEV_IMU, DEV_ULTRA, DEV_ULTRA0, DEV_ULTRA1, DEV_ULTRA2, DEV_ULTRA3, DEV_IR
  };
  return (slot < NUM_CFG) ? ids[slot] : 0xFF;
}

// --- CRC-8 (poly 0x07) ------------------------------------------------------
uint8_t DeviceConfig::crc8(const uint8_t *d, uint8_t n) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Serialise magic+ver+table+crc dans buf (>= EE_SIZE). Renvoie EE_SIZE.
uint8_t DeviceConfig::serialize(uint8_t *buf) const {
  uint8_t k = 0;
  buf[k++] = (uint8_t)(EE_MAGIC & 0xFF);
  buf[k++] = (uint8_t)(EE_MAGIC >> 8);
  buf[k++] = EE_LAYOUT;
  for (uint8_t i = 0; i < NUM_CFG; i++) {
    buf[k++] = _cfg[i].enabled;
    buf[k++] = _cfg[i].pinA;
    buf[k++] = _cfg[i].pinB;
    buf[k++] = (uint8_t)(_cfg[i].periodMs & 0xFF);
    buf[k++] = (uint8_t)(_cfg[i].periodMs >> 8);
  }
  buf[k] = crc8(buf, k);   // CRC sur tout ce qui precede
  k++;
  return k;                // = EE_SIZE
}

void DeviceConfig::loadDefaults() { fillDefaults(_cfg); }

void DeviceConfig::begin() {
  if (reload() != CFG_RES_DONE) loadDefaults();
}

// Recharge depuis l'EEPROM. Renvoie CFG_RES_DONE si valide, sinon CFG_RES_UNCHANGED
// (EEPROM vierge/corrompue -> l'appelant garde/charge les defauts).
uint8_t DeviceConfig::reload() {
  uint8_t raw[EE_SIZE];
  for (uint8_t i = 0; i < EE_SIZE; i++) raw[i] = EEPROM.read(EE_BASE + i);

  uint16_t magic = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);
  if (magic != EE_MAGIC || raw[2] != EE_LAYOUT) return CFG_RES_UNCHANGED;
  if (crc8(raw, EE_SIZE - 1) != raw[EE_SIZE - 1]) return CFG_RES_UNCHANGED;

  uint8_t k = 3;
  for (uint8_t i = 0; i < NUM_CFG; i++) {
    _cfg[i].enabled  = raw[k++];
    _cfg[i].pinA     = raw[k++];
    _cfg[i].pinB     = raw[k++];
    _cfg[i].periodMs = (uint16_t)raw[k] | ((uint16_t)raw[k + 1] << 8);
    k += 2;
  }
  return CFG_RES_DONE;
}

// Sauve la RAM en EEPROM avec protection anti-usure. Renvoie CFG_RES_*.
uint8_t DeviceConfig::save() {
  uint8_t buf[EE_SIZE];
  serialize(buf);

  // (b) skip total si identique a ce qui est deja stocke (compare le CRC stocke).
  uint8_t storedCrc = EEPROM.read(EE_BASE + EE_SIZE - 1);
  uint16_t storedMagic = (uint16_t)EEPROM.read(EE_BASE) |
                         ((uint16_t)EEPROM.read(EE_BASE + 1) << 8);
  if (storedMagic == EE_MAGIC && storedCrc == buf[EE_SIZE - 1])
    return CFG_RES_UNCHANGED;

  // (c) anti-rafale : refuse une ecriture trop rapprochee de la precedente.
  if (_hasWritten && (millis() - _lastWriteMs) < MIN_SAVE_INTERVAL_MS)
    return CFG_RES_THROTTLED;

  // (a) EEPROM.update() : n'ecrit physiquement que les octets reellement modifies.
  for (uint8_t i = 0; i < EE_SIZE; i++) EEPROM.update(EE_BASE + i, buf[i]);

  _lastWriteMs = millis();
  _hasWritten  = true;
  return CFG_RES_DONE;
}
