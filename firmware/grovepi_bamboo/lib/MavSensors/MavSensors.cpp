#include "MavSensors.h"

// Garde de compilation : PlatformIO (LDF) compile TOUS les .cpp de lib/MavSensors/
// quel que soit le transport actif. Sans ENABLE_MAVLINK (build trames binaires), le
// chemin d'inclusion du dialecte n'est pas fourni -> tout le corps est neutralise et
// cette unite de traduction devient vide. Meme motif que la STM32 (mav_protocol.c),
// l'ESP32 (ConnectorMavlink.cpp) et le Teensy (MavFrame.cpp).
#ifdef ENABLE_MAVLINK

#include <math.h>
#include "Protocol.h"       // CFG_ACT_* / CFG_RES_* / DIST_NONE / codes health
#include "DeviceConfig.h"   // SLOT_* + DevCfg + DeviceConfig

// Fonctions "convenience" MAVLink : mavlink_msg_*_send() serialise DIRECTEMENT sur
// l'UART via la macro ci-dessous, sans buffer TX statique (cf. MavSensors.h : SRAM
// de l'ATmega328P). Doit etre defini AVANT l'inclusion du dialecte.
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES(chan, buf, len) Serial.write((const uint8_t*)(buf), (size_t)(len))

// Adressage MAVLink (cf. bamboo.xml : sysid STM32=1 ESP32=2 Teensy=3 GrovePi=4).
#define MAV_SYS_ID_GROVE  (4)
#define MAV_COMP_ID       (1)   // MAV_COMP_ID_AUTOPILOT1

// PARTI-PRIS SRAM #2 (ATmega328P, 2 Ko) : la table `mavlink_message_crcs` du
// dialecte complet (common.xml, ~230 entrees) est un `static const` recopie EN RAM
// au demarrage (AVR : pas d'acces direct au flash) -> ~2100 octets, a lui seul plus
// que le budget SRAM entier. Or cette table ne sert QU'A LA RECEPTION
// (mavlink_get_msg_entry, appele par mavlink_parse_char pour retrouver longueur +
// CRC_EXTRA d'un msgid entrant) ; la table EST guardee par #ifndef dans bamboo.h.
// On la redefinit donc AVANT l'inclusion du dialecte, reduite aux SEULS messages
// que cette carte recoit reellement (PARAM_REQUEST_LIST/READ, PARAM_SET,
// COMMAND_LONG). Un msgid absent est simplement rejete par le parser (ce qu'on veut :
// on n'accepte rien d'autre). La TRANSMISSION _send() n'utilise pas cette table :
// son CRC_EXTRA vient de macros par-message (MAVLINK_MSG_ID_*_CRC) resolues a la
// compilation. Gain : ~2100 -> ~40 octets de SRAM.
//   entrees {msgid, crc_extra, min_len, max_len, flags, target_sys_ofs, target_comp_ofs}
#define MAVLINK_MESSAGE_CRCS { \
    {20, 214, 20, 20, 3, 2, 3},    /* PARAM_REQUEST_READ  */ \
    {21, 159, 2, 2, 3, 0, 1},      /* PARAM_REQUEST_LIST  */ \
    {23, 168, 23, 23, 3, 4, 5},    /* PARAM_SET           */ \
    {76, 152, 33, 33, 3, 30, 31} } /* COMMAND_LONG        */

// Identite emettrice pour les fonctions _send() : elles lisent ce global (pas un
// argument par appel). Le dialecte genere NE le declare PAS -> on inclut d'abord
// juste le type puis on DEFINIT le global, avant de tirer les helpers _send() qui
// le referencent. NUM_BUFFERS=1 est fixe par -DMAVLINK_COMM_NUM_BUFFERS=1.
#include "mavlink_types.h"
mavlink_system_t mavlink_system = { MAV_SYS_ID_GROVE, MAV_COMP_ID };
#include "bamboo/mavlink.h"

// --- Facteur d'echelle MPU6050 (config par defaut +-2 g / +-250 deg/s) ---
// ATTITUDE attend roll/pitch en radians ; le filtre complementaire les fournit
// en centi-degres (roll100/pitch100) -> conversion rad = deg100 * (PI/180) / 100.
static const float DEG100_TO_RAD    = (float)(M_PI / 180.0) / 100.0f;

// ======================================================================= //
//  Etat RX : parser MAVLink (mono-thread loop, aucun garde de concurrence). //
//  Pas d'etat TX : les _send() emettent directement sur l'UART.            //
// ======================================================================= //
static mavlink_message_t s_rx_msg;      // dernier message complet (copie de parse_char)
static mavlink_status_t  s_rx_status;

// ======================================================================= //
//  Table de parametres : image PARAM de la config DeviceConfig            //
//  (enable + periode par device). QGC edite ces params nativement ;        //
//  PREFLIGHT_STORAGE les persiste en EEPROM.                               //
// ======================================================================= //
enum {
    P_IMU_EN, P_IMU_MS,
    P_ULTRA_EN, P_ULTRA_MS,
    P_U0_EN, P_U1_EN, P_U2_EN, P_U3_EN,
    P_IR_EN, P_IR_MS,
    PARAM_COUNT
};

static const char * const s_param_name[PARAM_COUNT] = {
    "IMU_EN", "IMU_MS",
    "ULTRA_EN", "ULTRA_MS",
    "U0_EN", "U1_EN", "U2_EN", "U3_EN",
    "IR_EN", "IR_MS",
};

// param -> (slot, est-ce le champ periode ?). Retourne le slot, ou -1 si inconnu.
static int8_t paramSlot(uint16_t idx, bool &isPeriod) {
    isPeriod = false;
    switch (idx) {
    case P_IMU_EN:   return SLOT_IMU;
    case P_IMU_MS:   isPeriod = true; return SLOT_IMU;
    case P_ULTRA_EN: return SLOT_ULTRA;
    case P_ULTRA_MS: isPeriod = true; return SLOT_ULTRA;
    case P_U0_EN:    return SLOT_U0;
    case P_U1_EN:    return SLOT_U1;
    case P_U2_EN:    return SLOT_U2;
    case P_U3_EN:    return SLOT_U3;
    case P_IR_EN:    return SLOT_IR;
    case P_IR_MS:    isPeriod = true; return SLOT_IR;
    default:         return -1;
    }
}

// --- cycle de vie ---
void MavSensors::begin(DeviceConfig* cfg, ApplyCb apply, StorageCb storage) {
    cfg_       = cfg;
    applyCb_   = apply;
    storageCb_ = storage;
    // Serial deja demarre par setup() ; pas d'agent externe a joindre.
}

void MavSensors::poll() {
    // 1) vider le flux entrant et router les messages complets (CRC OK).
    while (Serial.available() > 0) {
        parseByte((uint8_t)Serial.read());
    }
    // 2) HEARTBEAT ~1 Hz (presence, type carte auxiliaire, etat actif).
    uint32_t now = millis();
    if (now - lastHeartbeat_ >= 1000) {
        lastHeartbeat_ = now;
        sendHeartbeat();
    }
}

// --- emission ---
void MavSensors::sendHeartbeat() {
    // MAV_TYPE_ONBOARD_CONTROLLER : carte auxiliaire (pas un vehicule). Autopilote
    // INVALID (composant non pilote). MAV_STATE_ACTIVE tant que la loop tourne.
    mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_ONBOARD_CONTROLLER,
                               MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
}

void MavSensors::emitImu(int16_t roll100, int16_t pitch100) {
    // ATTITUDE : roll/pitch (rad) du filtre complementaire. Pas de magneto (MPU6050)
    // -> yaw = 0 et rates non exposes par ce filtre -> 0 (yaw RELATIF, caveat MPU6050).
    // On n'emet PLUS SCALED_IMU (#26, accel/gyro bruts) : gyro non debiaise (aucune
    // calibration sur cette carte) donc valeurs tres bruitees, non consommees par
    // l'hote -> retirees de la telemetrie (economie de bande passante).
    mavlink_msg_attitude_send(MAVLINK_COMM_0, millis(),
                              roll100  * DEG100_TO_RAD,
                              pitch100 * DEG100_TO_RAD,
                              0.0f, 0.0f, 0.0f, 0.0f);
}

void MavSensors::emitDistance(uint8_t id, uint16_t mm, bool infrared) {
    // DISTANCE_SENSOR : distances en cm (le protocole maison portait des mm ; la
    // resolution cm est suffisante pour HC-SR04/Sharp et c'est le prix du standard).
    uint16_t mn, mx;
    uint8_t  type;
    if (infrared) { mn = 60; mx = 500; type = MAV_DISTANCE_SENSOR_INFRARED; }
    else          { mn = 2;  mx = 400; type = MAV_DISTANCE_SENSOR_ULTRASOUND; }
    bool     valid   = (mm != DIST_NONE);
    uint16_t cur     = valid ? (uint16_t)(mm / 10) : 0;   // 0 + quality 0 = pas de mesure
    uint8_t  quality = valid ? 100 : 0;
    float    q[4]    = { 0, 0, 0, 0 };
    mavlink_msg_distance_sensor_send(MAVLINK_COMM_0, millis(), mn, mx, cur,
                                     type, id, MAV_SENSOR_ROTATION_NONE,
                                     255 /* covariance inconnue */,
                                     0.0f, 0.0f, q, quality);
}

// ======================================================================= //
//  Parametres                                                             //
// ======================================================================= //
float MavSensors::paramGet(uint16_t idx) {
    bool isPeriod;
    int8_t s = paramSlot(idx, isPeriod);
    if (s < 0 || cfg_ == 0) return 0;
    const DevCfg &c = cfg_->slot((uint8_t)s);
    return isPeriod ? (float)c.periodMs : (float)c.enabled;
}

int MavSensors::paramSetByName(const char* name, float value) {
    uint16_t idx;
    for (idx = 0; idx < PARAM_COUNT; idx++) {
        // param_id MAVLink : jusqu'a 16 octets, non termine si plein.
        if (strncmp(name, s_param_name[idx], 16) == 0) break;
    }
    if (idx >= PARAM_COUNT || cfg_ == 0) return -1;

    bool isPeriod;
    int8_t s = paramSlot(idx, isPeriod);
    if (s < 0) return -1;
    DevCfg &c = cfg_->slot((uint8_t)s);
    if (isPeriod) c.periodMs = (uint16_t)value;
    else          c.enabled  = (value != 0.0f) ? 1 : 0;
    if (applyCb_ != 0) applyCb_((uint8_t)s);   // reconfig materielle (RAM seule)
    return (int)idx;
}

void MavSensors::sendParam(uint16_t idx) {
    if (idx >= PARAM_COUNT) return;
    mavlink_msg_param_value_send(MAVLINK_COMM_0, s_param_name[idx], paramGet(idx),
                                 MAV_PARAM_TYPE_REAL32, PARAM_COUNT, idx);
}

// ======================================================================= //
//  Commandes                                                              //
// ======================================================================= //
void MavSensors::sendCommandAck(uint16_t command, uint8_t result) {
    mavlink_msg_command_ack_send(MAVLINK_COMM_0, command, result, 0, 0,
                                 0, 0 /* target_system/component : broadcast */);
}

void MavSensors::handleCommandLong() {
    const mavlink_message_t *msg = &s_rx_msg;
    uint16_t cmd    = mavlink_msg_command_long_get_command(msg);
    float    p1     = mavlink_msg_command_long_get_param1(msg);
    uint8_t  result = MAV_RESULT_UNSUPPORTED;

    switch (cmd) {
    case MAV_CMD_PREFLIGHT_STORAGE: {
        // Convention MAVLink param1 : 0 = lire (reload), 1 = ecrire (save),
        // 2 = defauts. Remplace l'ancien CONFIG_ACTION 0x56.
        uint8_t action = (p1 == 1) ? CFG_ACT_SAVE
                       : (p1 == 2) ? CFG_ACT_DEFAULTS
                                   : CFG_ACT_RELOAD;
        uint8_t res = (storageCb_ != 0) ? storageCb_(action) : (uint8_t)CFG_RES_BADGUARD;
        result = (res == CFG_RES_DONE || res == CFG_RES_UNCHANGED)
                     ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED;
        break;
    }
    // Pas de recalibration gyro sur cette carte (filtre complementaire, aucun
    // biais explicite a re-mesurer) ni de bootloader logiciel (flash par ISP).
    case MAV_CMD_PREFLIGHT_CALIBRATION:
    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }
    sendCommandAck(cmd, result);
}

// ======================================================================= //
//  Reception : parsing + routage                                          //
// ======================================================================= //
void MavSensors::parseByte(uint8_t b) {
    if (mavlink_parse_char(MAVLINK_COMM_0, b, &s_rx_msg, &s_rx_status)) {
        // Trame complete + CRC OK : on route tout de suite (mono-thread).
        routeMessage();
    }
}

void MavSensors::routeMessage() {
    mavlink_message_t *msg = &s_rx_msg;
    switch (msg->msgid) {
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
        if (idx >= 0) sendParam((uint16_t)idx);   // echo de la valeur appliquee
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
