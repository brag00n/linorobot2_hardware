#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include "Connector.hpp"
#include <Arduino.h>
#include <string>
#include <iostream>

#include "odometry.h"
#include "imu_interface.h"
#include "mag_interface.h"
#include "battery.h"
#include "range.h"
#include "DeviceWifi.hpp"

//#define ENABLE_MICRO_ROS

#ifdef ENABLE_MICRO_ROS
// include all micro-ros library (used for PROD)
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/timer.h>
#include <rmw_microros/ping.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <micro_ros_utilities/string_utilities.h>

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

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*ErrorLoopCallback(temp_rc);*/}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif

#ifdef ENABLE_MICRO_ROS
#define ROS_EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t initRos = -1; \
    if (initRos == -1) { initRos = uxr_millis();} \
    if (uxr_millis() - initRos > MS) { X; initRos = uxr_millis();} \
  } while (0)
#else
#define ROS_EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t initRos = -1; \
    if (initRos == -1) { initRos = millis();} \
    if (millis() - initRos > MS) { X; initRos = millis();} \
  } while (0)
#endif



// #ifndef ERROR_LOOP_CALLBACK
// void ErrorLoopCallback(int error_code) {
//     // Handle the error code as needed, e.g., log it or take action
//     if (isSyslog) syslog(LOG_DEBUG,"Error: %d\n", error_code);
//     while (1) {
//         delay(1000); // Infinite loop to halt execution
//     }
// }
// #define ERROR_LOOP_CALLBACK
// #endif // ERROR_LOOP_CALLBACK

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
#ifndef RANGE_TIMER
#define RANGE_TIMER 100 // 10Hz
#endif

#define NR_OF_JOINTS 8 // number of joints

static Connector::connectorTwistCallbak_t twistCallback_=NULL;
static Connector::connectorJointCallbak_t jointCallback_=NULL;
static Connector::connectorPidCallbak_t pidCallback_=NULL;
static Connector::connectorTimerCallbak_t timerCallback_=NULL;

class ConnectorMicroROS : public Connector {

  private:
    rcl_publisher_t odom_publisher;
    rcl_publisher_t imu_publisher;
    rcl_publisher_t mag_publisher;
    rcl_subscription_t twist_subscriber;
    rcl_publisher_t battery_publisher;
    rcl_publisher_t range_publisher;

    rcl_subscription_t joint_subscriber;
    sensor_msgs__msg__JointState joint_msg[NR_OF_JOINTS];
    
    rclc_executor_t executor;
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;
    rcl_timer_t control_timer;

    nav_msgs__msg__Odometry odom_msg;
    sensor_msgs__msg__Imu imu_msg;
    sensor_msgs__msg__MagneticField mag_msg;
    geometry_msgs__msg__Twist twist_msg;
    sensor_msgs__msg__BatteryState battery_msg;
    sensor_msgs__msg__Range range_msg;
    sensor_msgs__msg__JointState joint_state_msg,req_state_msg;

  public:

    enum states{
      WAITING_AGENT =0,
      AGENT_AVAILABLE =1,
      AGENT_CONNECTED =2,
      AGENT_DISCONNECTED =3
    } state=WAITING_AGENT;

    struct micro_ros_agent_locator {
      IPAddress address;
      int port;
    };

    joint_state_t joint_state_[NR_OF_JOINTS];

    bool initAgent(const connectorTimerCallbak_t ptimerCallback,connectorTwistCallbak_t ptwistCallback,Connector::connectorJointCallbak_t pJointCallback,Connector::connectorPidCallbak_t pPidCallback);
    bool pingAgent(int timeout_ms, int attempts);
    bool isAvailable();
    bool listenAgent(long pWait_time_ms);
    void publishImu(IMUInterface::Imu_t pImu_msg);
    void publishOdom(Odometry::Odometry_data pOdom_msg);
    void publishMag(MAGInterface::Mag_t pMag_msg);
    void publishBattery(Battery::Battery_t pBattery_msg);
    void publishRange(Range::Range_t pRange);
    void publishWifi(DeviceWifi::DeviceWifi_t* pWifiData);
    void publishJoint(Connector::joint_state_t* pJointStateList);
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
    bool syncTime();
    struct timespec getTime();
    int64_t getMillis();

  private:
    static void controlCallback(rcl_timer_t * timer, int64_t last_call_time);
    static void twistCallback(const void * pTwist_msg);
    static void jointCallback(const void * pJointState_msg);

    // connectorTwistCallbak_t twistCallback_;
    // rclc_subscription_callback_t jointCallback_;
    // DeviceWifi::DeviceWifi_t deviceWifi_;
    DeviceWifi::DeviceWifi_t deviceWifi_;

    bool isAvailable(int timeout_ms, int attempts);
    bool destroyEntities();

    // used for joint state
    bool isPublishJointState=false;
    bool isPublishReqState=false;
    bool isPublish=false;
    double JointStateVelocity[NR_OF_JOINTS];
    double JointStatePosition[NR_OF_JOINTS];
    double ReqStateVelocity[NR_OF_JOINTS];
    double ReqStatePosition[NR_OF_JOINTS];


};

#ifdef ENABLE_MICRO_ROS
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
// remove wifi initialization code from wifi transport
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct ConnectorMicroROS::micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
#endif
#endif

#endif // #define ROS_COMMUNICATION_H