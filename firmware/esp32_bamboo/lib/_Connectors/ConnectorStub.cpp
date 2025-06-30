#include "ConnectorStub.hpp"
#include "MySyslog.h"

#define ROS_EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t initRos = -1; \
    if (initRos == -1) { initRos = millis();} \
    if (millis() - initRos > MS) { X; initRos = millis();} \
  } while (0)


void ErrorLoopCallback(int error_code) {
    // Handle the error code as needed, e.g., log it or take action
    if (isSyslog) syslog(LOG_DEBUG,"Error: %d\n", error_code);
    while (1) {
        delay(1000); // Infinite loop to halt execution
    }
}

void ConnectorStub::controlCallback(rcl_timer_t * timer, int64_t last_call_time){

}

void ConnectorStub::twistCallback(const void * msgin){
}


void ConnectorStub::jointCallback(const void *msgin){

}

bool ConnectorStub::initAgent(const rcl_timer_callback_t ptimerCallback,Connector::connectorCallbak_t ptwistCallback,connectorCallbak_t pJointCallback){
    
    if (isSyslog) syslog(LOG_DEBUG, "   ConnectorStub::initAgent\n");

    timerCallback_ = ptimerCallback;
    twistCallback_ = ptwistCallback; 
    jointCallback_ = pJointCallback;

    return true;
}

bool ConnectorStub::pingAgent(int timeout_ms, int attempts){
    return true;
}

bool ConnectorStub::listenAgent(long pWait_time_ms = 0){

    ROS_EXECUTE_EVERY_N_MS(100,
        if (isSyslog) syslog(LOG_DEBUG, "--- START listenAgent ---\n");
        if (timerCallback_ != NULL) {
            rcl_timer_t* pTimer = &control_timer; // Define pTimer as a pointer to the control_timer
            timerCallback_(pTimer, 0); // Pass appropriate arguments based on the function's definition
        }
        if (twistCallback_ != NULL) {
            twistCallback_(&twist_msg);
        }   
        if (jointCallback_ != NULL) {
            jointCallback_(&joint_msg);
        }
        if (isSyslog) syslog(LOG_DEBUG, "--- END listenAgent ---\n");
    );

    return true;
}

void ConnectorStub::setTwist(double linear_x, double linear_y, double angular_z){
    twist_msg_.linear_x = linear_x;
    twist_msg_.linear_y = linear_y;
    twist_msg_.angular_z = angular_z;
}
float ConnectorStub::getTwistX(){
    return twist_msg_.linear_x;
}
float ConnectorStub::getTwistY(){
    return twist_msg_.linear_y;
}
float ConnectorStub::getTwistZ(){
    return twist_msg_.angular_z;
}

void ConnectorStub::setImu(float ax, float ay, float az, float gx, float gy, float gz, float qx, float qy, float qz, float qw){
    struct timespec time_stamp = getTime();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;
    imu_msg.orientation.w = qw;

    #ifdef ENABLE_MICRO_ROS
    imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
    #endif
}

void ConnectorStub::setMag(float x, float y, float z){
    mag_msg.magnetic_field.x = x;
    mag_msg.magnetic_field.y = y;
    mag_msg.magnetic_field.z = z;
} 

void ConnectorStub::setOdometry(Odometry::Odometry_data pOdometry_data){
    struct timespec time_stamp = getTime();
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    //robot's position in x,y, and z
    odom_msg.pose.pose.position.x = pOdometry_data.pose_pose_position_x;
    odom_msg.pose.pose.position.y = pOdometry_data.pose_pose_position_y;
    odom_msg.pose.pose.position.z = pOdometry_data.pose_pose_position_z;

    //robot's heading in quaternion
    odom_msg.pose.pose.orientation.x = pOdometry_data.pose_pose_orientation_x;
    odom_msg.pose.pose.orientation.y = pOdometry_data.pose_pose_orientation_y;
    odom_msg.pose.pose.orientation.z = pOdometry_data.pose_pose_orientation_z;
    odom_msg.pose.pose.orientation.w = pOdometry_data.pose_pose_orientation_w;

    odom_msg.pose.covariance[0]=pOdometry_data.pose_covariance[0];
    odom_msg.pose.covariance[7]=pOdometry_data.pose_covariance[7];
    odom_msg.pose.covariance[14]=pOdometry_data.pose_covariance[14];
    odom_msg.pose.covariance[21]=pOdometry_data.pose_covariance[21];
    odom_msg.pose.covariance[28]=pOdometry_data.pose_covariance[28];
    odom_msg.pose.covariance[35]=pOdometry_data.pose_covariance[35];

    //linear speed from encoders
    odom_msg.twist.twist.linear.x = pOdometry_data.twist_twist_linear_x;
    odom_msg.twist.twist.linear.y=pOdometry_data.twist_twist_linear_y;
    odom_msg.twist.twist.linear.z=pOdometry_data.twist_twist_linear_z;

    //angular speed from encoders
    odom_msg.twist.twist.angular.x = pOdometry_data.twist_twist_angular_x;
    odom_msg.twist.twist.angular.y=pOdometry_data.twist_twist_angular_y;
    odom_msg.twist.twist.angular.z=pOdometry_data.twist_twist_angular_z;

    odom_msg.twist.covariance[0]=pOdometry_data.twist_covariance[0];
    odom_msg.twist.covariance[7]=pOdometry_data.twist_covariance[7];
    odom_msg.twist.covariance[14]=pOdometry_data.twist_covariance[14];
    odom_msg.twist.covariance[21]=pOdometry_data.twist_covariance[21];
    odom_msg.twist.covariance[28]=pOdometry_data.twist_covariance[28];
    odom_msg.twist.covariance[35]=pOdometry_data.twist_covariance[35];
}

void ConnectorStub::setBattery(Battery::Battery_t pBattery_msg){

    struct timespec time_stamp = getTime();
    battery_msg.header.stamp.sec = time_stamp.tv_sec;
    battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    battery_msg.voltage = pBattery_msg.voltage;
    battery_msg.current = pBattery_msg.current;
    battery_msg.percentage = pBattery_msg.percentage;
    battery_msg.capacity = pBattery_msg.capacity;
    battery_msg.design_capacity = pBattery_msg.design_capacity;
    battery_msg.present = pBattery_msg.present;
}

void ConnectorStub::setRange(float range){
    range_msg.range = range;
}
void ConnectorStub::setRange(float range, float min_range, float max_range){
    struct timespec time_stamp = getTime();
    range_msg.header.stamp.sec = time_stamp.tv_sec;
    range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    range_msg.range = range;
    range_msg.min_range = min_range;
    range_msg.max_range = max_range;
}

void ConnectorStub::setJointStateList(Connector::joint_state_t* pJointStateList){

    isPublishJointState=false;
    isPublishReqState=false;

    int listSize=sizeof(pJointStateList)/sizeof(pJointStateList[0]);
    joint_state_msg.name.size = joint_state_msg.position.size = joint_state_msg.velocity.size = listSize;
    for(int i=0, j=0; i<listSize; i++,j=i+4){

        //set joint [i] from velocity
#ifdef ENABLE_MICRO_ROS
        joint_state_msg.name.data[i] = micro_ros_string_utilities_set(joint_state_msg.name.data[i], pJointStateList[i].name.c_str());
#endif
        joint_state_msg.velocity.data[i]= pJointStateList[i].velocity * M_PI * 2 / 60; // rpm to rad/s
        joint_state_msg.position.data[i]+= joint_state_msg.velocity.data[i] / UPDATE_FREQ;  // rad/s to rad
        if (joint_state_msg.position.data[i] > M_PI){ joint_state_msg.position.data[i] = -1.0 * M_PI; }
        if (joint_state_msg.position.data[i] < -1.0 * M_PI){ joint_state_msg.position.data[i] = M_PI; }
        if (joint_state_msg.velocity.data[i] != JointStateVelocity[i] || 
            joint_state_msg.position.data[i] != JointStatePosition[i]){
           isPublishJointState=true;
           JointStateVelocity[i] = joint_state_msg.velocity.data[i]; 
           JointStatePosition[i] = joint_state_msg.position.data[i];
        }

        //set joint [i+4] from velocity_requested
#ifdef ENABLE_MICRO_ROS
        joint_state_msg.name.data[j] = micro_ros_string_utilities_set(joint_state_msg.name.data[j], (pJointStateList[j].name+"_requested").c_str());
#endif
        joint_state_msg.velocity.data[j]= pJointStateList[j].velocity_requested * M_PI * 2 / 60; // rpm to rad/s
        joint_state_msg.position.data[j]+= joint_state_msg.velocity.data[j] / UPDATE_FREQ;  // rad/s to rad
        if (joint_state_msg.position.data[j] > M_PI){ joint_state_msg.position.data[j] = -1.0 * M_PI; }
        if (joint_state_msg.position.data[j] < -1.0 * M_PI){ joint_state_msg.position.data[j] = M_PI; }
        if (joint_state_msg.velocity.data[j] != ReqStateVelocity[j] || 
            joint_state_msg.position.data[j] != ReqStatePosition[j]){
            isPublishReqState=true;
            ReqStateVelocity[j] = joint_state_msg.velocity.data[j]; 
            ReqStatePosition[j] = joint_state_msg.position.data[j];
        }
    }

    struct timespec time_stamp = getTime();
    joint_state_msg.header.stamp.sec = time_stamp.tv_sec;
}

void ConnectorStub::publishJoint(Connector::joint_state_t* pJointStateList){
    setJointStateList(pJointStateList);


    //if(isPublishJointState||isPublishReqState){
        if (isSyslog) syslog(LOG_DEBUG, "   Joints: \n");
        for(int i=0; i<NR_OF_JOINTS; i++){
            if ( pJointStateList[i].name != ""){
                if (isSyslog) syslog(LOG_DEBUG, ("      "+ String(pJointStateList[i].name) +": velocity="+String(pJointStateList[i].velocity) +",velocity_requested="+String(pJointStateList[i].velocity_requested) +"\n").c_str());
            }
        }
    //}
}

void ConnectorStub::publishRange(Range::Range_t pRange){
    setRange(pRange.range, pRange.min_range, pRange.max_range);

    if (isSyslog) syslog(LOG_DEBUG, ("   Range: range=" + String(range_msg.range) +"m\n").c_str());
}



bool ConnectorStub::syncTime(){
    return true;
}

struct timespec ConnectorStub::getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void ConnectorStub::publishImu(IMUInterface::Imu_t pImu_msg){

    setImu(pImu_msg.linear_acceleration.x, 
        pImu_msg.linear_acceleration.y, 
        pImu_msg.linear_acceleration.z,
        pImu_msg.angular_velocity.x, 
        pImu_msg.angular_velocity.y, 
        pImu_msg.angular_velocity.z,
        0.0, // Replace with a default or valid value for orientation.x
        0.0, // Replace with a default or valid value for orientation.y
        0.0, // Replace with a default or valid value for orientation.z
        0.0  // Replace with a default or valid value for orientation.w (1.0 ?)
    );

    if (isSyslog) syslog(LOG_DEBUG, ("   Imu: (x,y,z) lin.accel   =(" + String(imu_msg.linear_acceleration.x) +","+String(imu_msg.linear_acceleration.y)+","+String(imu_msg.linear_acceleration.z)+")\n").c_str());
    if (isSyslog) syslog(LOG_DEBUG, ("                ang.velo    =(" + String(imu_msg.angular_velocity.x) +","+String(imu_msg.angular_velocity.y)+","+String(imu_msg.angular_velocity.z)+")\n").c_str());
    if (isSyslog) syslog(LOG_DEBUG, ("                orientation =(" + String(imu_msg.orientation.x) +","+String(imu_msg.orientation.y)+","+String(imu_msg.orientation.z)+")\n").c_str());
}

void ConnectorStub::publishMag(MAGInterface::Mag_t pMag_msg){
    setMag(pMag_msg.magnetic_field.x, 
        pMag_msg.magnetic_field.y, 
        pMag_msg.magnetic_field.z);

    struct timespec time_stamp = getTime();
    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    if (isSyslog) syslog(LOG_DEBUG, ("   Mag: (x,y,z) mag.field   =(" + String(mag_msg.magnetic_field.x) +","+String(mag_msg.magnetic_field.y)+","+String(mag_msg.magnetic_field.z)+")\n").c_str());
}

void ConnectorStub::publishOdom(Odometry::Odometry_data pOdometry_data){
    setOdometry(pOdometry_data);

    struct timespec time_stamp = getTime();
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    if (isSyslog) syslog(LOG_DEBUG, ("   Odom: (x,y,z) twist.linear=(" + String(odom_msg.twist.twist.linear.x) +","+String(odom_msg.twist.twist.linear.y)+","+String(odom_msg.twist.twist.linear.z)+")\n").c_str());
}

void ConnectorStub::publishBattery(Battery::Battery_t pBattery_msg){
    setBattery(pBattery_msg);

    if (isSyslog) syslog(LOG_DEBUG, ("   Battery: voltage=" + String(battery_msg.voltage) +"V , current="+String(battery_msg.current)+"A ,percentage="+String(battery_msg.percentage)+"%\n").c_str());
}