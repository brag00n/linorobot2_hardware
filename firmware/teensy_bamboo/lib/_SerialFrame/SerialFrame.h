#ifndef SERIAL_FRAME_H
#define SERIAL_FRAME_H

// Couche de transport "trames binaires Bamboo" pour le Teensy, exclusive de micro-ROS
// (active par -D ENABLE_CONNECTOR_SERIAL_FRAME). Portage autonome et LEGER du coeur
// protocole de esp32_bamboo/lib/_Connectors/ConnectorSerialFrame.cpp : ici PAS de
// framework Connector/Device_t/*_msgs -- des signatures plates (float bruts) pour ne
// tirer AUCUN symbole rcl/rclc/rmw/micro_ros/uxr dans la branche trames.
//
// Trame : [0xFF][ID][LEN][FUNC][payload][CHK]
//   ID  = 0xFB carte -> hote (emission), 0xFC hote -> carte (reception)
//   LEN = 3 + taille(payload)
//   CHK = (LEN + FUNC + somme(payload)) & 0xFF
// Little-endian. Les trames de metriques auto-emises (0x0A/0x0C/0x0D) sont prefixees
// d'un timestamp horloge interne u32 LE ms (millis()) en tete de payload, comme la STM32.
//
// Le Teensy pilote ses moteurs via I2C (UGeek Motor HAT v2.0) et lit 4 encodeurs
// quadrature reels -> 0x0D porte de VRAIS comptages M1..M4. Pas de magneto (MPU6050)
// -> jamais de trame 0x0B. Servos differes.

#include <Arduino.h>

class SerialFrame {

  public:
    // Callbacks firmware : signatures plates (pas de Device_t / Twist_t ROS).
    typedef void (*TickCb)();                       // tick de controle ~10 Hz
    typedef void (*TwistCb)(float vx, float vy, float wz); // cmd_vel recue (m/s, rad/s)
    typedef void (*PidCb)(float kp, float ki, float kd);   // nouveaux gains PID moteur

    void begin(TickCb tick, TwistCb twist, PidCb pid); // Serial deja demarre par setup()
    void poll();                                       // vide Serial -> parseByte ; TickCb ~10 Hz

    // --- emission des metriques ---
    void emitSpeed(float vx, float vy, float wz, uint8_t battByte); // 0x0A
    void emitImu(float roll, float pitch, float yaw);               // 0x0C
    void emitEncoders(const int32_t m[4]);                          // 0x0D

    // --- geometrie / type de chassis (reponses aux requetes hote) ---
    void setWheelGeom(uint16_t cpr, float circ_mm, float apb_mm);   // reponse 0x16
    void setCarType(uint8_t t = 0x04);                              // reponse 0x15 (FOURWHEEL)

  private:
    // --- protocole ---
    static const uint8_t PTO_HEAD  = 0xFF;
    static const uint8_t PTO_ID_TX = 0xFB; // carte -> hote
    static const uint8_t PTO_ID_RX = 0xFC; // hote -> carte
    void sendFrame(uint8_t func, const uint8_t* payload, uint8_t len);
    void sendVersion();
    void replyRequest(uint8_t subfunc, uint8_t param);
    void parseByte(uint8_t b);
    void dispatch(uint8_t func, const uint8_t* params, uint8_t plen);

    // --- machine a etats du parseur d'entree ---
    enum ParseState { P_HEAD, P_ID, P_LEN, P_DATA };
    ParseState pstate_ = P_HEAD;
    uint8_t rxLen_  = 0; // octet LEN de la trame en cours
    uint8_t rxNeed_ = 0; // octets restants (FUNC + payload + CHK)
    uint8_t rxIdx_  = 0;
    uint8_t rxBuf_[64];

    // --- callbacks firmware ---
    TickCb  tickCb_  = NULL;
    TwistCb twistCb_ = NULL;
    PidCb   pidCb_   = NULL;

    // --- geometrie chassis (repond aux requetes 0x15/0x16) ---
    uint8_t  carType_ = 0x04;   // 0x04 = FOURWHEEL
    uint16_t cpr_     = 48;     // placeholder (a calibrer)
    uint16_t circMm10_= 0;      // circonference * 10 (mm)
    uint16_t apbMm10_ = 0;      // demi-voie/empattement * 10 (mm)

    // --- echo PID yaw (pas de controleur yaw : memorise pour repondre) ---
    float    yawP_ = 0, yawI_ = 0, yawD_ = 0;
    // --- echo PID moteur (dernier jeu recu, pour repondre a la requete) ---
    float    motP_ = 0, motI_ = 0, motD_ = 0;

    uint32_t lastTick_ = 0;   // cadence du tick de controle (~10 Hz)
};

#endif // SERIAL_FRAME_H
