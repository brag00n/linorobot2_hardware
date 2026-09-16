#ifndef MAV_FRAME_H
#define MAV_FRAME_H

// 3e mode de transport du Teensy : MAVLink v2 (dialecte bamboo), le protocole cible
// commun aux 3 cartes de controle (STM32 / ESP32 WaveShare / Teensy) et a l'hote
// robot_control. Selectionne a la compilation par -D ENABLE_MAVLINK, EXCLUSIF des
// deux autres transports sur le meme UART :
//   (aucun flag)                      -> micro-ROS (branche ENABLE_CONNECTOR_ROS)
//   -D ENABLE_CONNECTOR_SERIAL_FRAME  -> trames binaires maison (SerialFrame)
//   -D ENABLE_MAVLINK                 -> MAVLink v2 (ce module)
//
// Meme parti-pris que SerialFrame : portage AUTONOME et LEGER (signatures plates,
// floats bruts), sans framework Connector/*_msgs, pour ne tirer aucun symbole
// rcl/rclc/rmw dans cette branche. L'API publique est IDENTIQUE a SerialFrame
// (begin/poll/emitSpeed/emitImu/emitEncoders/setWheelGeom/setCarType) : le seam de
// firmware.ino est inchange, seul le codec du fil differe. Un unique setter en plus
// (setCalibrate) cable la recalibration gyro a la demande, propre a MAVLink.
//
// Adressage : sysid Teensy = 3, compid = 1 (MAV_COMP_ID_AUTOPILOT1). L'hote emet en
// sysid 255 ; QGroundControl voit une 3e carte "rover" distincte sur le banc.
//
// Telemetrie emise : HEARTBEAT (#0, ~1 Hz), SYS_STATUS (#1, tension batterie),
// ATTITUDE (#30, roll/pitch/yaw), BAMBOO_WHEEL_STATE (vx/vy/wz), BAMBOO_ENCODERS
// (4x int32). Pas de magneto (MPU6050) -> pas de BAMBOO_MAG.
// Commandes acceptees : BAMBOO_CMD_VEL, PARAM_*, COMMAND_LONG (dont
// PREFLIGHT_CALIBRATION -> recalibration du biais gyro, le besoin declencheur).

#include <Arduino.h>

class MavFrame {

  public:
    // Callbacks firmware : memes signatures plates que SerialFrame.
    typedef void (*TickCb)();                                // tick de controle ~10 Hz
    typedef void (*TwistCb)(float vx, float vy, float wz);   // cmd_vel recue (m/s, rad/s)
    typedef void (*PidCb)(float kp, float ki, float kd);     // nouveaux gains PID moteur
    typedef bool (*CalibCb)();                               // recalibration gyro -> true si OK

    void begin(TickCb tick, TwistCb twist, PidCb pid); // Serial deja demarre par setup()
    void poll();                                       // vide Serial ; HEARTBEAT 1 Hz ; TickCb ~10 Hz

    // --- emission des metriques (memes signatures que SerialFrame) ---
    void emitSpeed(float vx, float vy, float wz, uint8_t battByte); // BAMBOO_WHEEL_STATE + SYS_STATUS
    void emitImu(float roll, float pitch, float yaw);               // ATTITUDE
    void emitEncoders(const int32_t m[4]);                          // BAMBOO_ENCODERS

    // --- geometrie / type de chassis (echo via protocole PARAM) ---
    void setWheelGeom(uint16_t cpr, float circ_mm, float apb_mm);
    void setCarType(uint8_t t = 0x04);                              // 0x04 = FOURWHEEL

    // --- specifique MAVLink : cable la recalibration gyro (PREFLIGHT_CALIBRATION) ---
    void setCalibrate(CalibCb cb);

  private:
    // --- emission (pack MAVLink -> UART) ---
    void sendMessage();
    void sendHeartbeat();
    void sendParam(uint16_t idx);
    void sendCommandAck(uint16_t command, uint8_t result);

    // --- reception ---
    void parseByte(uint8_t b);   // alimente le parser MAVLink, route sur trame complete
    void routeMessage();         // dispatch du dernier message complet
    void handleCommandLong();

    // --- table de parametres (protocole PARAM, tous REAL32) ---
    float paramGet(uint16_t idx);
    int   paramSetByName(const char* name, float value); // -1 si inconnu

    // --- callbacks firmware ---
    TickCb  tickCb_  = NULL;
    TwistCb twistCb_ = NULL;
    PidCb   pidCb_   = NULL;
    CalibCb calibCb_ = NULL;

    // --- geometrie chassis (echo params, fige a la compilation) ---
    uint16_t cpr_     = 48;     // placeholder (a calibrer)
    float    circMm_  = 0;      // circonference (mm)
    float    apbMm_   = 0;      // demi-voie/empattement (mm)
    float    carType_ = 4.0f;   // 4 = FOURWHEEL

    // --- echo PID (un seul jeu partage par les 4 moteurs, comme onPid) ---
    float    motP_ = 0, motI_ = 0, motD_ = 0;
    float    yawP_ = 0, yawI_ = 0, yawD_ = 0; // pas de controleur yaw : memorise pour echo

    // --- caches / cadences ---
    uint16_t battMv_ = 0;         // tension mV, rafraichie par emitSpeed
    uint32_t lastTick_ = 0;       // cadence du tick de controle (~10 Hz)
    uint32_t lastHeartbeat_ = 0;  // cadence HEARTBEAT (~1 Hz)
};

#endif // MAV_FRAME_H
