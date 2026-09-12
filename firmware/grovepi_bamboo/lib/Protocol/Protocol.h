/*
 * Protocol.h - Trame serie "Bamboo" (miroir de la STM32) + codes partages.
 *
 * Trame little-endian :  [0xFF][ID][LEN][FUNC][donnees...][CHK]
 *   0xFF = entete ; ID = 0xFC hote->carte / 0xFB carte->hote ;
 *   LEN  = taille_totale - 2 ; CHK = somme(octets[2..fin-1]) & 0xFF.
 *
 * Ce module ne connait AUCUN capteur : il ne fait que (de)serialiser les trames.
 * L'orchestration (quelles trames, quand) vit dans src/main.cpp.
 */
#ifndef GROVEPI_PROTOCOL_H
#define GROVEPI_PROTOCOL_H

#include <Arduino.h>

// --- Constantes de trame ----------------------------------------------------
static const uint8_t PTO_HEAD  = 0xFF;
static const uint8_t PTO_ID_RX = 0xFC;   // hote -> carte
static const uint8_t PTO_ID_TX = 0xFB;   // carte -> hote

// --- Codes fonction (FUNC) --------------------------------------------------
static const uint8_t FUNC_REQUEST_DATA = 0x50;   // h->c : [subfunc][param]
static const uint8_t FUNC_VERSION      = 0x51;   // c->h : [major][minor][patch]
static const uint8_t FUNC_TIME_SYNC    = 0x52;   // h<->c : req [seq] ; ack [seq][t_board u32]
static const uint8_t FUNC_STATUS       = 0x53;   // c->h : [board_state][n]+n*[id][health]
static const uint8_t FUNC_CONFIG       = 0x54;   // c->h : dump config d'un device
static const uint8_t FUNC_SET_CONFIG   = 0x55;   // h->c : regle un device (RAM seule)
static const uint8_t FUNC_CONFIG_ACTION= 0x56;   // h<->c : save/defaults/reload EEPROM + ack
static const uint8_t FUNC_REPORT_IMU   = 0x60;   // c->h
static const uint8_t FUNC_REPORT_ULTRA = 0x61;   // c->h
static const uint8_t FUNC_REPORT_IR    = 0x62;   // c->h

// Garde d'ecriture EEPROM (convention STM32, PAS un FUNC).
static const uint8_t SAVE_VERIFY = 0x5F;

// --- Actions CONFIG_ACTION (0x56) -------------------------------------------
static const uint8_t CFG_ACT_SAVE     = 0x01;   // RAM -> EEPROM
static const uint8_t CFG_ACT_DEFAULTS = 0x02;   // reset usine (defauts compiles)
static const uint8_t CFG_ACT_RELOAD   = 0x03;   // recharge depuis EEPROM

// Resultats renvoyes dans l'ack 0x56.
static const uint8_t CFG_RES_DONE     = 0;   // ecrit / applique
static const uint8_t CFG_RES_UNCHANGED= 1;   // config identique, rien ecrit
static const uint8_t CFG_RES_THROTTLED= 2;   // ecriture trop rapprochee (anti-usure)
static const uint8_t CFG_RES_BADGUARD = 3;   // garde SAVE_VERIFY invalide

// --- Identifiants de device (dev_id) ----------------------------------------
static const uint8_t DEV_IMU     = 0x00;
static const uint8_t DEV_ULTRA   = 0x01;   // groupe (enable + period du batch 0x61)
static const uint8_t DEV_ULTRA0  = 0x10;   // canaux ultra 0..3 (enable + trig/echo)
static const uint8_t DEV_ULTRA1  = 0x11;
static const uint8_t DEV_ULTRA2  = 0x12;
static const uint8_t DEV_ULTRA3  = 0x13;
static const uint8_t DEV_IR      = 0x20;
static const uint8_t DEV_ALL     = 0xFF;   // param REQUEST_DATA = tous

// --- Codes de sante / erreur (octet "health" de STATUS et CONFIG) -----------
// 0x00 OK, 0x01 desactive (pas une erreur), >=0x10 => erreur reelle.
static const uint8_t HLTH_OK        = 0x00;
static const uint8_t HLTH_DISABLED  = 0x01;
static const uint8_t ERR_UNKNOWN    = 0x10;   // erreur generique
static const uint8_t ERR_I2C_NACK   = 0x11;   // IMU : pas d'ACK a l'adressage
static const uint8_t ERR_I2C_READ   = 0x12;   // IMU : lecture incomplete (<14 o)
static const uint8_t ERR_NO_ECHO    = 0x21;   // ultra : timeout persistant (absent/hors portee)
static const uint8_t ERR_ADC_LOW    = 0x31;   // IR : ADC bloque a 0 (debranche ?)
static const uint8_t ERR_ADC_HIGH   = 0x32;   // IR : ADC sature a 1023 (court-circuit ?)
inline bool healthIsError(uint8_t h) { return h >= 0x10; }

// --- Etat carte (board_state de STATUS) -------------------------------------
static const uint8_t BOARD_OK       = 0;
static const uint8_t BOARD_DEGRADED = 1;   // >=1 device active en erreur
static const uint8_t BOARD_FAULT    = 2;   // IMU (capteur principal) en erreur

// Valeur "pas de mesure" partagee (mm) / octet "garder" pour SET_CONFIG.
static const uint16_t DIST_NONE  = 0xFFFF;
static const uint8_t  CFG_KEEP8  = 0xFF;    // champ octet inchange dans SET_CONFIG

// --- (de)serialisation little-endian ----------------------------------------
inline void put16(uint8_t *b, uint16_t v) {
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
}
inline uint16_t get16(const uint8_t *b) {
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}
inline void put32(uint8_t *b, uint32_t v) {
  b[0] = (uint8_t)(v & 0xFF);
  b[1] = (uint8_t)((v >> 8) & 0xFF);
  b[2] = (uint8_t)((v >> 16) & 0xFF);
  b[3] = (uint8_t)((v >> 24) & 0xFF);
}
inline uint32_t get32(const uint8_t *b) {
  return (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

// Emet une trame complete [HEAD][ID_TX][LEN][FUNC][data..][CHK] sur Serial.
void sendFrame(uint8_t func, const uint8_t *data, uint8_t n);

// --- Machine a etats de reception (hote -> carte) ---------------------------
// Accepte les octets un par un ; appelle onFrame(func, data, n) a chaque trame
// valide (checksum OK). Tolere le bruit, resync sur l'entete.
class FrameParser {
public:
  typedef void (*Handler)(uint8_t func, const uint8_t *data, uint8_t n);
  explicit FrameParser(Handler h) : _h(h), _len(0) {}
  void feed(uint8_t b);
private:
  Handler _h;
  uint8_t _buf[24];
  uint8_t _len;
};

#endif // GROVEPI_PROTOCOL_H
