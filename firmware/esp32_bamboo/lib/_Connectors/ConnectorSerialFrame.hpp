#ifndef CONNECTOR_SERIAL_FRAME_H
#define CONNECTOR_SERIAL_FRAME_H

// 4e mode de transport : protocole de trames binaires Bamboo, identique a celui
// parle par la carte STM32 (YahBoom) et la carte capteurs GrovePi. Permet a l'app
// robot_control / robot_controlv3 et aux MCP (qui ne comprennent QUE ce protocole)
// de trouver, lire et piloter la carte ESP32 WaveShare sans micro-ROS ni HTTP.
//
// Trame : [0xFF][ID][LEN][FUNC][payload][CHK]
//   ID  = 0xFB carte -> hote (emission), 0xFC hote -> carte (reception)
//   LEN = 3 + taille(payload)
//   CHK = (LEN + FUNC + somme(payload)) & 0xFF   (= somme(trame[2:]) & 0xFF)
// Little-endian. Les trames de metriques auto-emises (0x0A/0x0C/0x0D) sont
// prefixees d'un timestamp horloge interne u32 LE ms (millis()) en tete de payload,
// exactement comme la STM32 (donnees decalees de TS_LEN=4).
//
// Exclusif de micro-ROS (meme UART @921600) ; active par -D ENABLE_CONNECTOR_SERIAL_FRAME.
// Le pilotage servo ST3215 (bus serie) est HORS PERIMETRE initial.

#include "Connector.hpp"
#include <Arduino.h>

class ConnectorSerialFrame : public Connector {

  public:
    bool initAgent(const connectorTimerCallbak_t ptimerCallback,
                   connectorTwistCallbak_t ptwistCallback,
                   connectorJointCallbak_t pJointCallback,
                   connectorPidCallbak_t pPidCallback);
    bool isAvailable();
    bool pingAgent(int timeout_ms, int attempts);
    bool listenAgent(long pWait_time_ms);
    void publishImu(IMUInterface::Imu_t pImu_msg);
    void publishOdom(Odometry::Odometry_data pOdom_msg);
    void publishMag(MAGInterface::Mag_t pMag_msg);
    void publishBattery(Battery::Battery_t pBattery_msg);
    void publishRange(Range::Range_t pRange);
    void publishJoint(joint_state_t* pJointStateList);
    void publishWifi(DeviceWifi::DeviceWifi_t* pWifiData);
    void setTwist(double linear_x, double linear_y, double angular_z);
    float getTwistX();
    float getTwistY();
    float getTwistZ();
    void setImu(float ax, float ay, float az, float gx, float gy, float gz, float qx, float qy, float qz, float qw);
    void setBattery(Battery::Battery_t pBattery_msg);
    void setMag(float x, float y, float z);
    void setOdometry(Odometry::Odometry_data pOdom_msg);
    void setRange(float range);
    void setRange(float range, float min_range, float max_range);
    void setJointStateList(joint_state_t* pJointStateList);
    bool syncTime();
    struct timespec getTime();
    int64_t getMillis(); // utilise par EXECUTE_EVERY_N_MS dans firmware.cpp

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
    connectorTimerCallbak_t timerCallback_ = NULL;
    connectorTwistCallbak_t twistCallback_ = NULL;
    connectorJointCallbak_t jointCallback_ = NULL;
    connectorPidCallbak_t   pidCallback_   = NULL;

    // --- caches de metriques ---
    uint8_t  battByte_ = 0;   // tension * 10, mis a jour par publishBattery
    float    yawP_ = 0, yawI_ = 0, yawD_ = 0; // pas de controleur yaw : memorise pour echo
    uint32_t lastTick_ = 0;   // cadence du tick de controle (~10 Hz)
};

#endif // CONNECTOR_SERIAL_FRAME_H
