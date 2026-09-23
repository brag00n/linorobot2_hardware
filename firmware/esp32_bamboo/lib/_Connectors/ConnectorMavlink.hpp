#ifndef CONNECTOR_MAVLINK_H
#define CONNECTOR_MAVLINK_H

// 5e mode de transport : MAVLink v2 (dialecte bamboo), le protocole cible commun
// aux 3 cartes de controle (STM32 / ESP32 WaveShare / Teensy) et a l'hote
// robot_control. Remplace a terme le protocole de trames binaires maison
// (ConnectorSerialFrame) ; les deux restent selectionnables a la compilation :
//   -D ENABLE_CONNECTOR_SERIAL_FRAME  -> trames binaires (historique)
//   -D ENABLE_MAVLINK                 -> MAVLink v2 (ce connecteur)
//   (aucun des deux)                  -> micro-ROS (ConnectorMicroROS)
//
// Le transport reste l'UART @921600 partage (donc exclusif de micro-ROS). On
// reutilise le symbole connectorROS dans firmware.cpp : toute la logique de
// controle (moveBase / publishData / callbacks) est inchangee, seul le codec du
// fil differe. L'interface Connector est preservee telle quelle.
//
// Adressage : sysid ESP32 = 2, compid = 1 (MAV_COMP_ID_AUTOPILOT1). L'hote emet
// en sysid 255 ; QGroundControl voit une carte "rover" distincte.
//
// Telemetrie emise : HEARTBEAT (#0, ~1 Hz), SYS_STATUS (#1, tension batterie),
// ATTITUDE (#30, roll/pitch/yaw), BAMBOO_WHEEL_STATE (vx/vy/wz),
// BAMBOO_ENCODERS (4x int32), BAMBOO_MAG (mx/my/mz uT, AK09918C).
// Commandes acceptees : BAMBOO_CMD_VEL, PARAM_*, COMMAND_LONG.
//
// Identite du microcode : "ESP32-WROOM-32UE_bamboo vX.Y.Z" (include/fw_version.h),
// retournee en STATUSTEXT (#253) + AUTOPILOT_VERSION (#148), une fois au
// demarrage puis sur MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES.

#include "Connector.hpp"
#include <Arduino.h>

class ConnectorMavlink : public Connector {

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

    // Recalibrage gyro a la demande : le handler MAV_CMD_PREFLIGHT_CALIBRATION pose un
    // drapeau (ACK immediat, sans bloquer le parsing serie), consomme par la boucle
    // firmware qui lance le recalibrage ~2 s sur l'objet IMU (que ce connecteur ne
    // possede pas). Retourne true UNE fois si une demande etait en attente, puis la
    // rearme a false.
    bool takeGyroCalRequest();

  private:
    // --- emission (pack MAVLink -> UART) ---
    void sendMessage();               // serialise s_tx_msg_ vers Serial
    void sendHeartbeat();
    void sendParam(uint16_t idx);
    void sendCommandAck(uint16_t command, uint8_t result);
    void sendVersion();               // STATUSTEXT + AUTOPILOT_VERSION (identite microcode)

    // --- reception ---
    void parseByte(uint8_t b);        // alimente le parser MAVLink, route sur trame complete
    void routeMessage();              // dispatch du dernier message complet (rxReady_)
    void handleCommandLong();

    // --- table de parametres (protocole PARAM, tous REAL32) ---
    float  paramGet(uint16_t idx);
    int    paramSetByName(const char* name, float value); // -1 si inconnu

    // --- callbacks firmware ---
    connectorTimerCallbak_t timerCallback_ = NULL;
    connectorTwistCallbak_t twistCallback_ = NULL;
    connectorJointCallbak_t jointCallback_ = NULL;
    connectorPidCallbak_t   pidCallback_   = NULL;

    // --- caches de metriques ---
    uint16_t battMv_   = 0;   // tension mV, rafraichie par publishBattery
    float    yawP_ = 0, yawI_ = 0, yawD_ = 0; // pas de controleur yaw : memorise pour echo
    uint32_t lastTick_ = 0;   // cadence du tick de controle (~10 Hz)
    uint32_t lastHeartbeat_ = 0; // cadence HEARTBEAT (~1 Hz)
    volatile bool gyroCalReq_ = false; // demande de recalibrage gyro en attente
    bool versionSent_ = false; // banniere d'identite deja emise (une fois au demarrage)
};

#endif // CONNECTOR_MAVLINK_H
