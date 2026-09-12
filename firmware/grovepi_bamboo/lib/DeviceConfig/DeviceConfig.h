/*
 * DeviceConfig.h - Configuration des devices (RAM) + persistance EEPROM protegee.
 *
 * Table indexee par "slot" (un device = un slot). Chaque slot :
 *   enabled : 0/1 (active) ; pinA/pinB : broches selon le device ; periodMs : cadence.
 *   IMU        -> pinA = adresse I2C (0x68), pinB inutilise, periodMs = report.
 *   ULTRA (grp)-> pinA/pinB inutilises, periodMs = cadence du batch 0x61.
 *   ULTRA0..3  -> pinA = Trig, pinB = Echo, periodMs inutilise.
 *   IR         -> pinA = broche analogique (A0), pinB inutilise, periodMs = report.
 *
 * EEPROM : [magic u16][layout_ver u8][NUM_CFG * 5 o serialises][crc u8].
 * save() PROTEGE l'EEPROM contre les rafales : (a) EEPROM.update() n'ecrit que
 * les octets modifies ; (b) skip total si le CRC RAM == CRC stocke ; (c) intervalle
 * minimal (throttle millis) entre deux ecritures physiques. Renvoie un CFG_RES_*.
 */
#ifndef GROVEPI_DEVICECONFIG_H
#define GROVEPI_DEVICECONFIG_H

#include <Arduino.h>
#include "Protocol.h"

struct DevCfg {
  uint8_t  enabled;
  uint8_t  pinA;
  uint8_t  pinB;
  uint16_t periodMs;
};

// Slots (ordre = ordre de dump STATUS). NUM_CFG entrees.
enum {
  SLOT_IMU = 0,
  SLOT_ULTRA,          // groupe
  SLOT_U0, SLOT_U1, SLOT_U2, SLOT_U3,
  SLOT_IR,
  NUM_CFG
};

class DeviceConfig {
public:
  void    begin();                       // charge EEPROM si valide, sinon defauts
  void    loadDefaults();                // (re)ecrit la table RAM avec les defauts compiles
  uint8_t reload();                       // recharge depuis EEPROM -> CFG_RES_*
  uint8_t save();                         // RAM -> EEPROM (protege) -> CFG_RES_*

  DevCfg&       slot(uint8_t i)       { return _cfg[i]; }
  const DevCfg& slot(uint8_t i) const { return _cfg[i]; }

  static int8_t slotOf(uint8_t devId);    // dev_id -> index slot, -1 si inconnu
  static uint8_t devIdOf(uint8_t slot);   // index slot -> dev_id

private:
  uint8_t  serialize(uint8_t *buf) const; // ecrit magic+ver+cfg+crc, renvoie taille
  static uint8_t crc8(const uint8_t *d, uint8_t n);

  DevCfg   _cfg[NUM_CFG];
  uint32_t _lastWriteMs = 0;
  bool     _hasWritten  = false;
};

#endif // GROVEPI_DEVICECONFIG_H
