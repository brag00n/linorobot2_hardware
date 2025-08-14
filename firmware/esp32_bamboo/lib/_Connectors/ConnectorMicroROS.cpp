#include "ConnectorMicroROS.hpp"
#include "MySyslog.h"


void ConnectorMicroROS::controlCallback(rcl_timer_t * timer, int64_t last_call_time){
    // execute timer callback
    if (timerCallback_ != NULL) {
        timerCallback_(reinterpret_cast<Connector::timer_t*>(timer), last_call_time,"ConnectorMicroROS");
    }
}

void ConnectorMicroROS::twistCallback(const void* ppTwist_msg){

    const geometry_msgs__msg__Twist *pTwist_msg = reinterpret_cast<const geometry_msgs__msg__Twist *>(ppTwist_msg);
    Twist_t pTwist_msg_;
    pTwist_msg_.linear_x = pTwist_msg->linear.x;
    pTwist_msg_.linear_y = pTwist_msg->linear.y;
    pTwist_msg_.angular_z = pTwist_msg->angular.z;

    if(twistCallback_ != NULL)
        twistCallback_(&pTwist_msg_,"ConnectorMicroROS");
}


void ConnectorMicroROS::jointCallback(const void * pJointState_msg){
    const sensor_msgs__msg__JointState *joint_msg = reinterpret_cast<const sensor_msgs__msg__JointState *>(pJointState_msg);
    // if (jointCallback_ != NULL) {
    //     jointCallback_(joint_state_list, "ConnectorMicroROS");
    // }
}

bool ConnectorMicroROS::initAgent(const connectorTimerCallbak_t ptimerCallback,connectorTwistCallbak_t ptwistCallback,Connector::connectorJointCallbak_t pJointCallback,Connector::connectorPidCallbak_t pPidCallback){
    
    if (isSyslog) syslog(LOG_DEBUG, "   ConnectorMicroROS::initAgent\n");
    
    timerCallback_ = ptimerCallback;
    twistCallback_ = ptwistCallback; 
    jointCallback_ = pJointCallback;
    pidCallback_ = pPidCallback;

    if (state != AGENT_AVAILABLE)  return true;

#ifdef ENABLE_MICRO_ROS

    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    // if we have magnetomter, use imu/data_raw for madgwick filter
#ifndef USE_FAKE_MAG
        TOPIC_PREFIX "imu/data_raw"
#else
        TOPIC_PREFIX "imu/data"
#endif
    ));
#ifndef USE_FAKE_MAG
    RCCHECK(rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        TOPIC_PREFIX "imu/mag"
    ));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219)
    // create battery pyblisher
    RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    TOPIC_PREFIX "battery"
    ));
#endif
#ifdef ECHO_PIN
    // create range pyblisher
    RCCHECK(rclc_publisher_init_default(
    &range_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    TOPIC_PREFIX "sonar"
    ));
#endif
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"
    ));

#ifdef JOINT_STATE_SUBSCRIBER
    // create joint command subscriber
    RCCHECK(rclc_subscription_init_default(
        &joint_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        TOPIC_PREFIX JOINT_STATE_SUBSCRIBER
    ));
#endif

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = CONTROL_TIMER;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        ConnectorMicroROS::controlCallback
    ));
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        ConnectorMicroROS::twistCallback,
        ON_NEW_DATA
    ));
#ifdef JOINT_STATE_SUBSCRIBER
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &joint_subscriber,
        &joint_msg,
        ConnectorMicroROS::jointCallback,
        ON_NEW_DATA
    ));

    // initialize measured joint state message memory
    micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_state_msg,
        (micro_ros_utilities_memory_conf_t) {});
     log(1,"   Connected to topic joint_state");
 
     // initialize required joint state message memory
     micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &req_state_msg,
        (micro_ros_utilities_memory_conf_t) {});
     log(1,"   Connected to topic req_state");

     
// #ifdef JOINT_STATE_SUBSCRIBER
//     // allocate dynamic msg memory
//     static micro_ros_utilities_memory_conf_t conf = {0};
//     conf.max_string_capacity = 20;
//     conf.max_ros2_type_sequence_capacity = 10;
//     conf.max_basic_type_sequence_capacity = 10;
//     bool success = micro_ros_utilities_create_message_memory(
//         ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
//         &joint_msg,
//         conf
//     );
//     if (isSyslog) syslog(LOG_DEBUG, "%s %lu allocate msg %d", __FUNCTION__, millis(), success);
// #endif

     // populate fixed message fields - size, frame ID and joint names for measured joint state
     joint_state_msg.header.frame_id = micro_ros_string_utilities_set(joint_state_msg.header.frame_id, BASE_FRAME_ID);
     joint_state_msg.name.size = joint_state_msg.position.size = joint_state_msg.velocity.size = NR_OF_JOINTS;
     joint_state_msg.name.data[0] = micro_ros_string_utilities_set(joint_state_msg.name.data[0], MOTOR1);
     joint_state_msg.name.data[1] = micro_ros_string_utilities_set(joint_state_msg.name.data[1], MOTOR2);
     joint_state_msg.name.data[2] = micro_ros_string_utilities_set(joint_state_msg.name.data[2], MOTOR3);
     joint_state_msg.name.data[3] = micro_ros_string_utilities_set(joint_state_msg.name.data[3], MOTOR4);
 
     // populate fixed message fields - size, frame ID and joint names for required joint state
     req_state_msg.name.size = req_state_msg.position.size = req_state_msg.velocity.size = NR_OF_JOINTS;
     req_state_msg.header.frame_id = micro_ros_string_utilities_set(req_state_msg.header.frame_id, BASE_FRAME_ID);
     req_state_msg.name.data[0] = micro_ros_string_utilities_set(req_state_msg.name.data[0], MOTOR1);
     req_state_msg.name.data[1] = micro_ros_string_utilities_set(req_state_msg.name.data[1], MOTOR2);
     req_state_msg.name.data[2] = micro_ros_string_utilities_set(req_state_msg.name.data[2], MOTOR3);
     req_state_msg.name.data[3] = micro_ros_string_utilities_set(req_state_msg.name.data[3], MOTOR4);


#endif
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
    IPAddress agent_ip;
    agent_ip.fromString(AGENT_IP);
    set_microros_net_transports(agent_ip, std::stoi(AGENT_PORT));
#else
    set_microros_serial_transports(Serial);
#endif


#endif  // #ifdef ENABLE_MICRO_ROS

    syncTime();
    return true;
}

bool ConnectorMicroROS::isAvailable(){
    return (state==AGENT_CONNECTED);
}

bool ConnectorMicroROS::isAvailable(int timeout_ms, int attempts){
    #ifdef ENABLE_MICRO_ROS
    return (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts));
    #else
    return true;
    #endif
}
bool ConnectorMicroROS::pingAgent(int timeout_ms, int attempts){

    switch (state)
    {
        case WAITING_AGENT:         
            ROS_EXECUTE_EVERY_N_MS(10000, 
                if (isSyslog) syslog(LOG_DEBUG, "   check ROS agent availability\n"); 
                state = (isAvailable(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;
                if (state == WAITING_AGENT) {
                    if (isSyslog) syslog(LOG_INFO, "   ROS agent unavailable\n");
                }
            );
            return false;
            break;
        case AGENT_AVAILABLE:{
            bool initROSOK=initAgent(timerCallback_,twistCallback_,jointCallback_, pidCallback_); // initialize ROS agent
            bool timeOK=   syncTime(); // synchronize time with the agent

            state = (initROSOK&&timeOK) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) destroyEntities();
            if (state == AGENT_CONNECTED) {
                if (isSyslog) syslog(LOG_INFO, "   ROS agent connected\n");
             }
            return false;
            break;
        }
        case AGENT_CONNECTED:
            ROS_EXECUTE_EVERY_N_MS(200, 
                state = (isAvailable(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            return (state==AGENT_CONNECTED);
            break;
        case AGENT_DISCONNECTED:
            if (isSyslog) syslog(LOG_INFO, "   ROS agent disconnected\n");

            destroyEntities();
            state = WAITING_AGENT;
            return false;
            break;
        default:
            break;
    }
    return false;
}

bool ConnectorMicroROS::destroyEntities(){

    if (isSyslog) syslog(LOG_INFO, "   ConnectorMicroROS::destroy agent entities");

#ifdef ENABLE_MICRO_ROS
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
#ifndef USE_FAKE_MAG
    RCSOFTCHECK(rcl_publisher_fini(&mag_publisher, &node));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219)
    RCSOFTCHECK(rcl_publisher_fini(&battery_publisher, &node));
#endif
#ifdef ECHO_PIN
    RCSOFTCHECK(rcl_publisher_fini(&range_publisher, &node));
#endif
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
#ifdef JOINT_STATE_SUBSCRIBER
    RCSOFTCHECK(rcl_subscription_fini(&joint_subscriber, &node));
#endif
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

#endif

    return true;
}

bool ConnectorMicroROS::listenAgent(long pWait_time_ms = 0){
#ifdef ENABLE_MICRO_ROS
    if (pWait_time_ms > 0) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(pWait_time_ms));
    } else {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
#else

    ROS_EXECUTE_EVERY_N_MS(100,
        if (timerCallback_ != NULL) {
            if (isSyslog) syslog(LOG_DEBUG, "--- START ConnectorMicroROS::listenAgent timerCallback ---\n");
            rcl_timer_t pTimer;
            controlCallback(&pTimer, 0); // Pass appropriate arguments based on the function's definition
            if (isSyslog) syslog(LOG_DEBUG, "--- END ConnectorMicroROS::listenAgent timerCallbackHello  ---\n");
        }

        if (twistCallback_ != NULL) {
            // if (twist_previous_msg_.linear_x != twist_msg_.linear_x ||
            //     twist_previous_msg_.linear_y != twist_msg_.linear_y ||
            //     twist_previous_msg_.angular_z != twist_msg_.angular_z) {
                if (isSyslog) syslog(LOG_DEBUG, ("   ConnectorMicroROS::listenAgent Twist:{x:" + String(twist_msg_.linear_x) + ",y:" + String(twist_msg_.linear_y) + ",z:" + String(twist_msg_.angular_z) + "}\n").c_str());
                twistCallback_(&twist_msg_,"ConnectorMicroROS");
            //     twist_previous_msg_.linear_x = twist_msg_.linear_x;
            //     twist_previous_msg_.linear_y = twist_msg_.linear_y;
            //     twist_previous_msg_.angular_z = twist_msg_.angular_z;
            // }
        }   
        // if (jointCallback_ != NULL) {
        //     jointCallback_(joint_state_list, "ConnectorMicroROS");
        // }
        if (pidCallback_ != NULL) {
            pidCallback_(&Pid_,"ConnectorMicroROS");
        }
    );

#endif
    return true;
}

void ConnectorMicroROS::setTwist(double linear_x, double linear_y, double angular_z){
    twist_msg.linear.x = linear_x;
    twist_msg.linear.y = linear_y;
    twist_msg.angular.z = angular_z;
}
float ConnectorMicroROS::getTwistX(){
    return twist_msg.linear.x;
}
float ConnectorMicroROS::getTwistY(){
    return twist_msg.linear.y;
}
float ConnectorMicroROS::getTwistZ(){
    return twist_msg.angular.z;
}

void ConnectorMicroROS::setImu(float ax, float ay, float az, float gx, float gy, float gz, float qx, float qy, float qz, float qw){
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

void ConnectorMicroROS::setMag(float x, float y, float z){
    mag_msg.magnetic_field.x = x;
    mag_msg.magnetic_field.y = y;
    mag_msg.magnetic_field.z = z;
} 

void ConnectorMicroROS::setOdometry(Odometry::Odometry_data pOdometry_data){
    #ifdef ENABLE_MICRO_ROS
    odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
    odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");
    #endif

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

void ConnectorMicroROS::setBattery(Battery::Battery_t pBattery_msg){

    struct timespec time_stamp = getTime();
    battery_msg.header.stamp.sec = time_stamp.tv_sec;
    battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    battery_msg.voltage = pBattery_msg.voltage;
    battery_msg.current = pBattery_msg.current;
    battery_msg.percentage = pBattery_msg.percentage;
    battery_msg.capacity = pBattery_msg.capacity;
    battery_msg.design_capacity = pBattery_msg.design_capacity;
    battery_msg.present = pBattery_msg.present;

    #ifdef ENABLE_MICRO_ROS
    battery_msg.header.frame_id = micro_ros_string_utilities_set(battery_msg.header.frame_id, "battery");
    #endif
}

void ConnectorMicroROS::setRange(float range){
    range_msg.range = range;
}
void ConnectorMicroROS::setRange(float range, float min_range, float max_range){
    struct timespec time_stamp = getTime();
    range_msg.header.stamp.sec = time_stamp.tv_sec;
    range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    range_msg.range = range;
    range_msg.min_range = min_range;
    range_msg.max_range = max_range;

    #ifdef ENABLE_MICRO_ROS
    range_msg.header.frame_id = micro_ros_string_utilities_set(range_msg.header.frame_id, "range");
    #endif
}

void ConnectorMicroROS::setJointStateList(Connector::joint_state_t* pJointStateList){

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
#ifdef ENABLE_MICRO_ROS
    joint_state_msg.header.frame_id = micro_ros_string_utilities_set(joint_state_msg.header.frame_id, "joint_states");
#endif
}

void ConnectorMicroROS::publishJoint(Connector::joint_state_t* pJointStateList){
    setJointStateList(pJointStateList);

    #ifdef ENABLE_MICRO_ROS
    RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL));
    #endif

    //if(isPublishJointState||isPublishReqState){
        if (isSyslog) syslog(LOG_DEBUG, "   Joints: \n");
        for(int i=0; i<NR_OF_JOINTS; i++){
            if ( pJointStateList[i].name != ""){
                if (isSyslog) syslog(LOG_DEBUG, ("      "+ String(pJointStateList[i].name) +": velocity="+String(pJointStateList[i].velocity) +",velocity_requested="+String(pJointStateList[i].velocity_requested) +"\n").c_str());
            }
        }
    //}
}

void ConnectorMicroROS::publishWifi(DeviceWifi::DeviceWifi_t* pWifiData){
    deviceWifi_.id = pWifiData->id;
    deviceWifi_.name = pWifiData->name;
    deviceWifi_.mode = pWifiData->mode;
    deviceWifi_.ssid = pWifiData->ssid;
    deviceWifi_.ipAddress = pWifiData->ipAddress;
    deviceWifi_.macAddress = pWifiData->macAddress;
    deviceWifi_.rssi = pWifiData->rssi;

    if (isSyslog) syslog(LOG_DEBUG, ("   Wifi: ssid=" + deviceWifi_.ssid + ", ssid=" + deviceWifi_.ssid + ", rssi=" + String(deviceWifi_.rssi) + "dBm\n").c_str());
}

void ConnectorMicroROS::publishRange(Range::Range_t pRange){
    setRange(pRange.range, pRange.min_range, pRange.max_range);

    #ifdef ENABLE_MICRO_ROS
    RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL));
    #endif

    if (isSyslog) syslog(LOG_DEBUG, ("   Range: range=" + String(range_msg.range) +"m\n").c_str());
}

bool ConnectorMicroROS::syncTime(){
#ifdef ENABLE_MICRO_ROS
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
    timespec tp;
    tp.tv_sec = time_ns / 1000000000;
    tp.tv_nsec = time_ns % 1000000000;
    clock_settime(CLOCK_REALTIME, &tp);
#else
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - millis();
#endif
    return true;
    }
    return false;
#else
    return true;
#endif

}

int64_t ConnectorMicroROS::getMillis(){
#ifdef ENABLE_MICRO_ROS
    return uxr_millis();
#else
    return millis();
#endif
}

struct timespec ConnectorMicroROS::getTime()
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

void ConnectorMicroROS::publishImu(IMUInterface::Imu_t pImu_msg){

    setImu(pImu_msg.linear_acceleration.x, 
        pImu_msg.linear_acceleration.y, 
        pImu_msg.linear_acceleration.z,
        pImu_msg.angular_velocity.x, 
        pImu_msg.angular_velocity.y, 
        pImu_msg.angular_velocity.z,
        pImu_msg.orientation.x, // Replace with a default or valid value for orientation.x
        pImu_msg.orientation.y, // Replace with a default or valid value for orientation.y
        pImu_msg.orientation.z, // Replace with a default or valid value for orientation.z
        pImu_msg.orientation.w  // Replace with a default or valid value for orientation.w (1.0 ?)
    );

    #ifdef ENABLE_MICRO_ROS
    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    #endif

    if (isSyslog) syslog(LOG_DEBUG, ("   Imu: (x,y,z) lin.accel   =(" + String(imu_msg.linear_acceleration.x) +","+String(imu_msg.linear_acceleration.y)+","+String(imu_msg.linear_acceleration.z)+")\n").c_str());
    if (isSyslog) syslog(LOG_DEBUG, ("                ang.velo    =(" + String(imu_msg.angular_velocity.x) +","+String(imu_msg.angular_velocity.y)+","+String(imu_msg.angular_velocity.z)+")\n").c_str());
    if (isSyslog) syslog(LOG_DEBUG, ("                orientation =(" + String(imu_msg.orientation.x) +","+String(imu_msg.orientation.y)+","+String(imu_msg.orientation.z)+")\n").c_str());
}

void ConnectorMicroROS::publishMag(MAGInterface::Mag_t pMag_msg){
    setMag(pMag_msg.magnetic_field.x, 
        pMag_msg.magnetic_field.y, 
        pMag_msg.magnetic_field.z);

    struct timespec time_stamp = getTime();
    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    #ifdef ENABLE_MICRO_ROS
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
    #endif

    if (isSyslog) syslog(LOG_DEBUG, ("   Mag: (x,y,z) mag.field   =(" + String(mag_msg.magnetic_field.x) +","+String(mag_msg.magnetic_field.y)+","+String(mag_msg.magnetic_field.z)+")\n").c_str());
}

void ConnectorMicroROS::publishOdom(Odometry::Odometry_data pOdometry_data){
    setOdometry(pOdometry_data);

    struct timespec time_stamp = getTime();
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    #ifdef ENABLE_MICRO_ROS
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    #endif

    if (isSyslog) syslog(LOG_DEBUG, ("   Odom: (x,y,z) twist.linear=(" + String(odom_msg.twist.twist.linear.x) +","+String(odom_msg.twist.twist.linear.y)+","+String(odom_msg.twist.twist.linear.z)+")\n").c_str());
}

void ConnectorMicroROS::publishBattery(Battery::Battery_t pBattery_msg){
    setBattery(pBattery_msg);
    if (isSyslog) syslog(LOG_DEBUG, ("   publishBattery START: voltage=" + String(battery_msg.voltage) +"V , current="+String(battery_msg.current)+"A ,percentage="+String(battery_msg.percentage)+"%\n").c_str());

    #ifdef ENABLE_MICRO_ROS
    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    #endif

    if (isSyslog) syslog(LOG_DEBUG, ("   publishBattery END: voltage=" + String(battery_msg.voltage) +"V , current="+String(battery_msg.current)+"A ,percentage="+String(battery_msg.percentage)+"%\n").c_str());
}

