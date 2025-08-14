#ifndef CONNNECTOR_H
#define CONNNECTOR_H

#include <string>
#include "Device.hpp"
#include "odometry.h"
#include "battery.h"
#include "range.h"
#include "kinematics.h"
#include "encoder.h"
#include "motor.h"
#include "imu_interface.h"
#include "mag_interface.h"
#include "DeviceWifi.hpp"
//#include <rcl/rcl.h>

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif

#define NR_OF_JOINTS 8 // number of joints

class Connector {

  protected:

  public:
    struct joint_state_t: Device::Device_t {
        String name;
        float position;
        float velocity;
        float effort;
        float velocity_requested;
        float position_requested;
        float effort_requested;
    } joint_state_[NR_OF_JOINTS];
  
    struct Twist_t: Device::Device_t {
        float linear_x;
        float linear_y;
        float angular_z;
    } twist_msg_,twist_previous_msg_;

    struct Pid_t: Device::Device_t {
        float P;
        float I;
        float D;
    } Pid_;

    struct DeviceWifi_t: Device::Device_t {
        String ssid;
        String bssid;
        int32_t rssi;
        String mode;
        String channel;
        String encryption_type;
    } wifi_msg_;

    typedef struct timer_impl_s timer_impl_t;
    /// Structure which encapsulates a ROS Timer.
    typedef struct timer_s
    {
    /// Private implementation pointer.
    timer_impl_t * impl;
    } timer_t;

    typedef void (* connectorTimerCallbak_t)(timer_t * timer, int64_t last_call_time,String source);
    typedef void (* connectorCallbak_t)(const void* pMsg, String pSource);
    typedef void (* connectorTwistCallbak_t)(const Connector::Twist_t* pTwist, String pSource);
    typedef void (* connectorJointCallbak_t)(const Connector::joint_state_t* pJointState, String pSource);
    typedef void (* connectorPidCallbak_t)(const Connector::Pid_t* pPid, String pSource);

    Odometry::Odometry_data odom_msg;
    Battery::Battery_t battery_msg;
    MAGInterface::Mag_t mag_msg;
    Range::Range_t range_msg;

    virtual bool initAgent(const connectorTimerCallbak_t pCallback,Connector::connectorTwistCallbak_t ptwistCallback,Connector::connectorJointCallbak_t pJointCallback,Connector::connectorPidCallbak_t pPidCallback) =0;
    virtual bool isAvailable() =0;
    virtual bool pingAgent(int timeout_ms, int attempts) =0;
    virtual bool listenAgent(long pWait_time_ms)=0;
    virtual void publishImu(IMUInterface::Imu_t pImu_msg)=0;
    virtual void publishOdom(Odometry::Odometry_data pOdom_msg) = 0;
    virtual void publishMag(MAGInterface::Mag_t pMag_msg)=0;
    virtual void publishBattery(Battery::Battery_t pBattery_msg)=0;
    virtual void publishRange(Range::Range_t pRange) = 0;
    virtual void publishJoint(joint_state_t* pJointStateList)=0;
    virtual void publishWifi(DeviceWifi::DeviceWifi_t* pWifiData)=0;
    virtual void setTwist(double linear_x, double linear_y, double angular_z) = 0;
    virtual float getTwistX() = 0;
    virtual float getTwistY() = 0;
    virtual float getTwistZ() = 0;
    virtual void setPID(float P, float I, float D) {
        Pid_.P = P;
        Pid_.I = I;
        Pid_.D = D;
    };
    virtual Connector::Pid_t getPID() {
        return Pid_;
    };
    virtual void setImu(float ax, float ay, float az, float gx, float gy, float gz, float qx, float qy, float qz, float qw) = 0;
    virtual void setBattery(Battery::Battery_t pBattery_msg) = 0;
    virtual void setMag(float x, float y, float z) = 0;
    virtual void setOdometry(Odometry::Odometry_data pOdom_msg) = 0;
    virtual void setRange(float range) = 0;
    virtual void setRange(float range, float min_range, float max_range) = 0;
    virtual void setJointStateList(joint_state_t* pJointStateList)=0;
    virtual bool syncTime() = 0;
    virtual timespec getTime() = 0;
    
#ifndef ENABLE_MICRO_ROS
//   private:
//     Connector::connectorCallbak_t twistCallback_;
//     Connector::connectorCallbak_t jointCallback_;
//     Connector::connectorCallbak_t pidCallback_;
//     Connector::connectorTimerCallbak_t timerCallback_;
#endif
};

#endif // #define CONNNECTOR_H