#ifndef CONNECTORWEB_H
#define CONNECTORWEB_H

#include "Connector.hpp"
#include <Arduino.h>
#include "odometry.h"
#include "imu_interface.h"
#include "mag_interface.h"
#include "battery.h"
#include "range.h"
#include "web_page.h"
//#include "wifi_ctrl.h"
#include "DeviceWifi.hpp"
#include <WebServer.h>
#include <ArduinoJson.h>
#include "MySyslog.h"


//#define ENABLE_MICRO_ROS

#ifdef ENABLE_MICRO_ROS
// include all micro-ros library (used for PROD)
#include <micro_ros_platformio.h>
#include <rclc/timer.h>

#else
// include ros scructures (used for TEST)
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/ping.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#endif

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // 50Hz
#endif
#ifndef BATTERY_TIMER
#define BATTERY_TIMER 2000 // 2 sec
#endif

#define NR_OF_JOINTS 8 // number of joints

class ConnectorWeb : public Connector {

  private:
    // rcl_publisher_t odom_publisher;
    // rcl_publisher_t imu_publisher;
    // rcl_publisher_t mag_publisher;
    // rcl_subscription_t twist_subscriber;
    // rcl_publisher_t battery_publisher;
    // rcl_publisher_t range_publisher;

    // rcl_subscription_t joint_subscriber;
    // sensor_msgs__msg__JointState joint_msg;
    
    // rclc_executor_t executor;
    // rclc_support_t support;
    // rcl_allocator_t allocator;
    // rcl_node_t node;
    rcl_timer_t control_timer;
    IMUInterface::Imu_t imu_msg;
    Connector::joint_state_t joint_state_msg;


    // nav_msgs__msg__Odometry odom_msg;
    // sensor_msgs__msg__Imu imu_msg;
    // sensor_msgs__msg__MagneticField mag_msg;
    // sensor_msgs__msg__BatteryState battery_msg;
    // sensor_msgs__msg__Range range_msg;
    // sensor_msgs__msg__JointState joint_state_msg,req_state_msg;

  public:
    joint_state_t joint_state_[NR_OF_JOINTS];

    bool initAgent(const connectorTimerCallbak_t pCallback,connectorTwistCallbak_t ptwistCallback,Connector::connectorJointCallbak_t pJointCallback,Connector::connectorPidCallbak_t pPidCallback);
    bool isAvailable();
    bool pingAgent(int timeout_ms, int attempts);
    bool listenAgent(long pWait_time_ms);
    void publishImu(IMUInterface::Imu_t pImu_msg);
    void publishOdom(Odometry::Odometry_data pOdom_msg);
    void publishMag(MAGInterface::Mag_t pMag_msg);
    void publishBattery(Battery::Battery_t pBattery_msg);
    void publishRange(Range::Range_t pRange);
    void publishJoint(Connector::joint_state_t* pJointStateList);
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
    void setJointStateList(Connector::joint_state_t* pJointStateList);
    void setKinematics(Kinematics* pKinematics) {
        pKinematics_ = pKinematics;
    };
    bool syncTime();
    struct timespec getTime();
  private:
      Kinematics* pKinematics_=NULL;
//#ifndef ENABLE_MICRO_ROS
    Connector::connectorTwistCallbak_t twistCallback_;
    Connector::connectorJointCallbak_t jointCallback_;
    Connector::connectorPidCallbak_t pidCallback_;
    Connector::connectorTimerCallbak_t timerCallback_;
    //connectorTimerCallbak_t timerCallback_;
    String jsonFeedbackWeb = "";
    JsonDocument jsonCmdReceive;
    JsonDocument jsonInfoSend;
    JsonDocument jsonInfoHttp;
    Battery::Battery_t battery_msg;
    DeviceWifi::DeviceWifi_t deviceWifi_;

    // used for joint state
    void controlCallback(rcl_timer_t * timer, int64_t last_call_time);
    void twistCallback(const void * msgin);
    void jointCallback(const void *msgin);
    bool isPublishJointState=false;
    bool isPublishReqState=false;
    bool isPublish=false;
    double JointStateVelocity[NR_OF_JOINTS];
    double JointStatePosition[NR_OF_JOINTS];
    double ReqStateVelocity[NR_OF_JOINTS];
    double ReqStatePosition[NR_OF_JOINTS];
//#endif

};

#endif // CONNECTORWEB_H