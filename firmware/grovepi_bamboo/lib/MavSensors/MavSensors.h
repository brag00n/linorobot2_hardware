#ifndef MAV_SENSORS_H
#define MAV_SENSORS_H

// Transport MAVLink v2 (dialecte bamboo) de la carte capteurs GrovePi+, en
// remplacement des trames binaires maison (lib/Protocol). Selectionne a la
// compilation par -D ENABLE_MAVLINK, EXCLUSIF du transport trames (main.cpp est
// gardee par #ifndef ENABLE_MAVLINK, main_mavlink.cpp par #ifdef ENABLE_MAVLINK).
//
// Adressage : sysid GrovePi = 4, compid = 1 (MAV_COMP_ID_AUTOPILOT1). L'hote emet
// en sysid 255. Les 3 cartes de controle sont sysid 1/2/3 -> pas de collision.
//
// PARTI-PRIS MEMOIRE (ATmega328P, 2 Ko de SRAM) : contrairement aux templates
// STM32/Teensy/ESP32 qui packent dans un mavlink_message_t puis serialisent a la
// main, ce module utilise les fonctions "convenience" _send() de MAVLink qui
// emettent DIRECTEMENT sur l'UART (aucun buffer TX statique) et fixe
// MAVLINK_COMM_NUM_BUFFERS=1 (un seul canal) -> empreinte SRAM ~divisee par deux.
// La contrepartie est l'identite emettrice via le global `mavlink_system`
// (defini dans le .cpp) au lieu d'un argument sysid par appel.
//
// Telemetrie emise :
//   HEARTBEAT (#0, ~1 Hz)      : presence, MAV_TYPE_ONBOARD_CONTROLLER (carte auxiliaire)
//   ATTITUDE (#30)             : roll/pitch (rad) du filtre complementaire  <- ex 0x60
//   DISTANCE_SENSOR (#132) x5  : 4 HC-SR04 (id 0..3) + 1 Sharp IR (id 4)    <- ex 0x61/0x62
// (SCALED_IMU #26 accel/gyro bruts retire : gyro non calibre -> bruite, inutilise cote hote.)
// Commandes acceptees :
//   PARAM_REQUEST_LIST/READ, PARAM_SET : table DeviceConfig (enable/periode)  <- ex 0x54/0x55
//   COMMAND_LONG PREFLIGHT_STORAGE     : commit/defauts/reload EEPROM         <- ex 0x56

#include <Arduino.h>

class DeviceConfig;   // declaration avant : le .cpp inclut DeviceConfig.h

class MavSensors {
  public:
    // Callbacks firmware (implementes dans main_mavlink.cpp).
    typedef void    (*ApplyCb)(uint8_t slot);       // reconfigure le materiel d'un slot
    typedef uint8_t (*StorageCb)(uint8_t action);   // EEPROM save/defaults/reload -> CFG_RES_*

    void begin(DeviceConfig* cfg, ApplyCb apply, StorageCb storage);
    void poll();   // vide l'UART entrant + route ; HEARTBEAT ~1 Hz

    // --- emission de la telemetrie (appelee par la loop, miroir des sendImu/Ultra/Ir) ---
    void emitImu(int16_t roll100, int16_t pitch100);           // ATTITUDE (roll/pitch)
    void emitDistance(uint8_t id, uint16_t mm, bool infrared); // DISTANCE_SENSOR (mm->cm)

  private:
    // --- emission ---
    void sendHeartbeat();
    void sendParam(uint16_t idx);
    void sendCommandAck(uint16_t command, uint8_t result);

    // --- reception ---
    void parseByte(uint8_t b);   // alimente le parser MAVLink, route sur trame complete
    void routeMessage();         // dispatch du dernier message complet
    void handleCommandLong();

    // --- table de parametres (protocole PARAM, tous REAL32) ---
    float paramGet(uint16_t idx);
    int   paramSetByName(const char* name, float value);  // -1 si inconnu

    DeviceConfig* cfg_       = 0;
    ApplyCb       applyCb_   = 0;
    StorageCb     storageCb_ = 0;
    uint32_t      lastHeartbeat_ = 0;   // cadence HEARTBEAT (~1 Hz)
};

#endif // MAV_SENSORS_H
