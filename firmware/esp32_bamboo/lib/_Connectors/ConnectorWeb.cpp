#include "ConnectorWeb.hpp"
#define FEEDBACK_BASE_INFO  1001
#define FEEDBACK_IMU_DATA   1002
// esp-now recv
// {"T":1003,"mac":"FF:FF:FF:FF:FF:FF","megs":"hello!"}
#define CMD_ESP_NOW_RECV 1003
// esp-now send status
// 0:failed   1:succeed   2:Error initializing ESP-NOW
// 3:invalid MAC address format.
// 4:Failed to add peer.
// 5:add peer.   6:delete peer.
// 7:error sending the data.   8:sent with success.
// {"T":1004,"mac":"FF:FF:FF:FF:FF:FF","status":1,"megs":"xxx"}
#define CMD_ESP_NOW_SEND 1004
// bus servos error feedback
// {"T":1005,"id":1,"status":1}
#define CMD_BUS_SERVO_ERROR 1005

#define M_PIl          3.141592653589793238462643383279502884L /* pi */

WebServer webServer(80);

void handleRoot(){
  webServer.send(200, "text/html", index_html); //Send web page
  if (isSyslog) syslog(LOG_INFO, "   ConnectorWeb: request recieved\n");
}

#define WEB_EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t initRos = -1; \
    if (initRos == -1) { initRos = millis();} \
    if (millis() - initRos > MS) { X; initRos = millis();} \
  } while (0)


bool ConnectorWeb::initAgent(const connectorTimerCallbak_t ptimerCallback,connectorTwistCallbak_t ptwistCallback,Connector::connectorJointCallbak_t pJointCallback,Connector::connectorPidCallbak_t pPidCallback){
    
    if (isSyslog) syslog(LOG_DEBUG, "   ConnectorWeb::initAgent\n");

    ConnectorWeb::timerCallback_ = ptimerCallback;
    ConnectorWeb::twistCallback_ = ptwistCallback; 
    ConnectorWeb::jointCallback_ = pJointCallback;
    ConnectorWeb::pidCallback_ = pPidCallback;


    webServer.on("/", handleRoot);

    webServer.on("/ping", [](){
        webServer.send(200, "text/plain", "this works as well");
    });


    webServer.on("/js", [this](){
        String jsonCmdWebString = webServer.arg(0);
        deserializeJson(jsonCmdReceive, jsonCmdWebString);
        String macString ="";
        bool isUnknown = false;
        int command=jsonCmdReceive["T"].as<int>();
        if (webServer.client().available()>5) {
            if (isSyslog) syslog(LOG_INFO, "   ConnectorWeb::initAgent: webServer.client().available() > 100\n");
            webServer.send(400, "text/plain", "Request too large");
            webServer.client().flush(); // clear the client buffer
            return;
        }
        //if (isSyslog) syslog(LOG_INFO, ("   ConnectorWeb: CmdReceive[T]=" + String(jsonCmdReceive["T"].as<int>()) + "\n").c_str());
	    switch(command) {
            case 1: // INPUT DATA request
            {
                    float l=0,r=0;
                    if (!jsonCmdReceive["L"].isNull()) l = jsonCmdReceive["L"].as<float>()*pKinematics_->getMaxRPM();
                    if (!jsonCmdReceive["R"].isNull()) r = jsonCmdReceive["R"].as<float>()*pKinematics_->getMaxRPM();
                    Kinematics::velocities vel = pKinematics_->getVelocities(l,r,l,r);
                    twist_msg_.linear_x = vel.linear_x;
                    twist_msg_.linear_y = vel.linear_y;
                    twist_msg_.angular_z = vel.angular_z;
                    if (!(jsonCmdReceive["L"].isNull() || jsonCmdReceive["R"].isNull())){
                        if (ConnectorWeb::twistCallback_ != NULL) { 
                            if (isSyslog) syslog(LOG_DEBUG, ("    ConnectorWeb::listenAgent (on js cmd 1): cmd:{L:" + String(jsonCmdReceive["L"].as<float>()) + ",R:" + String(jsonCmdReceive["R"].as<float>()) + "},Twist:{x:" + String(twist_msg_.linear_x) + ",y:" + String(twist_msg_.linear_y) + ",z:" + String(twist_msg_.angular_z) + "}\n").c_str());
                            ConnectorWeb::twistCallback_(&twist_msg_,"ConnectorWeb");
                        }
                    }
                break;
            }
            case 2: //
                if (isSyslog) syslog(LOG_DEBUG, ("    ConnectorWeb::listenAgent (on js cmd 2): Pid_:{x:" + String(Pid_.P) + ",y:" + String(Pid_.I) + ",z:" + String(Pid_.D) + "}\n").c_str());
                if (!jsonCmdReceive["P"].isNull()) Pid_.P = jsonCmdReceive["P"].as<float>();
                if (!jsonCmdReceive["I"].isNull()) Pid_.I = jsonCmdReceive["I"].as<float>();
                if (!jsonCmdReceive["D"].isNull()) Pid_.D = jsonCmdReceive["D"].as<float>();
                if (!(jsonCmdReceive["P"].isNull() || jsonCmdReceive["I"].isNull() || jsonCmdReceive["D"].isNull())){
                    if (ConnectorWeb::pidCallback_ != NULL) {
                        if (isSyslog) syslog(LOG_DEBUG, ("    ConnectorWeb::listenAgent (on js cmd 2): Pid_:{x:" + String(Pid_.P) + ",y:" + String(Pid_.I) + ",z:" + String(Pid_.D) + "}\n").c_str());
                        ConnectorWeb::pidCallback_(&Pid_,"ConnectorWeb");
                    }
                }
                break;
            case 130: // OUTPUT DATA request
                jsonInfoHttp.clear();
                jsonInfoHttp["T"] = FEEDBACK_BASE_INFO;

                jsonInfoHttp["L"] = joint_state_[0].velocity;
                jsonInfoHttp["R"] = joint_state_[1].velocity;

                jsonInfoHttp["r"] = imu_msg.linear_acceleration.x;
                jsonInfoHttp["p"] = imu_msg.linear_acceleration.y;
                jsonInfoHttp["y"] = imu_msg.linear_acceleration.z;

                jsonInfoHttp["q0"] = imu_msg.angular_velocity.x;
                jsonInfoHttp["q1"] = imu_msg.angular_velocity.y;
                jsonInfoHttp["q2"] = imu_msg.angular_velocity.z;
                jsonInfoHttp["q3"] = imu_msg.orientation.x;

                jsonInfoHttp["v"] = battery_msg.voltage;

                jsonInfoHttp["pan"]  = joint_state_[0].velocity;
		        jsonInfoHttp["tilt"] = joint_state_[1].velocity;

                if (isSyslog) syslog(LOG_INFO, ("   ConnectorWeb::listenAgent CmdReceive[T]=" + String(jsonCmdReceive["T"].as<int>()) + ", Response={L:" + String(jsonInfoHttp["L"].as<float>()) + ",R:" + String(jsonInfoHttp["R"].as<float>()) + ",pan:" + String(jsonInfoHttp["pan"].as<float>()) + ",tilt:" + String(jsonInfoHttp["tilt"].as<float>()) +  ",v:" + String(jsonInfoHttp["v"].as<float>()) + "}\n").c_str());
                break;
            case 405: // OUTPUT DATA request
                jsonInfoHttp.clear();
                jsonInfoHttp["T"] = FEEDBACK_BASE_INFO;
                jsonInfoHttp["mac"]= deviceWifi_.macAddress;
                jsonInfoHttp["ip"] = deviceWifi_.ipAddress;
                jsonInfoHttp["rssi"] = deviceWifi_.rssi;

                if (isSyslog) syslog(LOG_DEBUG, ("   ConnectorWeb::listenAgent CmdReceive[T]=" + String(jsonCmdReceive["T"].as<int>()) + ", Response={mac:" + deviceWifi_.macAddress + ",ip:" + deviceWifi_.ipAddress + ",rssi:" + String(deviceWifi_.rssi) + "}\n").c_str());
                break;
            case 600: // OUTPUT DATA request
                jsonInfoHttp.clear();
                if (isSyslog) syslog(LOG_INFO, ("   ConnectorWeb::listenAgent CmdReceive[T]=" + String(jsonCmdReceive["T"].as<int>()) + ", Restart requested\n").c_str());
                ESP.restart();
                break;
            default:
                isUnknown = true;
                break;
        }
        if (!isUnknown){
            if (isSyslog) syslog(LOG_DEBUG, ("   ConnectorWeb::listenAgent CmdReceive[T]=" + String(jsonCmdReceive["T"].as<int>()) + ", Response=200\n").c_str());
            serializeJson(jsonInfoHttp, jsonFeedbackWeb);
            webServer.send(200, "text/plain", jsonFeedbackWeb);
        }else{
            if (isSyslog) syslog(LOG_DEBUG, ("   ConnectorWeb::listenAgent CmdReceive[T]=" + String(jsonCmdReceive["T"].as<int>()) + ", Response=404\n").c_str());
            webServer.send(404, "text/plain", "404: Not found");
        }
        jsonFeedbackWeb = "";
        jsonInfoHttp.clear();
        jsonCmdReceive.clear();
    });

    webServer.onNotFound([](){
        if (isSyslog) syslog(LOG_DEBUG, "   ConnectorWeb::listenAgent 404 Not Found\n");
        webServer.send(404, "text/plain", "404: Not found");
    });

    // webServer.on("/js", [](){
    //     String jsonCmdWebString = webServer.arg(0);
    //     deserializeJson(jsonCmdReceive, jsonCmdWebString);
    //     jsonCmdReceiveHandler();
    //     serializeJson(jsonInfoHttp, jsonFeedbackWeb);
    //     webServer.send(200, "text/plane", jsonFeedbackWeb);
    //     jsonFeedbackWeb = "";
    //     jsonInfoHttp.clear();
    //     jsonCmdReceive.clear();
    // });

    // Start webServer
    webServer.begin();
    if (isSyslog) syslog(LOG_DEBUG, "   Web webServer Started.");
    
    return true;
}

bool ConnectorWeb::isAvailable(){
    return WiFi.status() == WL_CONNECTED; // Check if WiFi is connected
}

bool ConnectorWeb::pingAgent(int timeout_ms, int attempts){
    return true;
}

bool ConnectorWeb::listenAgent(long pWait_time_ms = 0){

    webServer.handleClient();
    WEB_EXECUTE_EVERY_N_MS(100,
        if (isSyslog) syslog(LOG_DEBUG, "--- START listenAgent ---\n");
        if (ConnectorWeb::timerCallback_ != NULL) {
            Connector::timer_t control_timer;
            ConnectorWeb::timerCallback_(&control_timer, 0,"ConnectorWeb"); // Pass the correct type pointer directly
        }
        // if (twistCallback_ != NULL) {
        //         if (isSyslog) syslog(LOG_DEBUG, "    Calling twistCallback_...\n");
        //         if (isSyslog) syslog(LOG_INFO, ("    ConnectorWeb::listenAgent Twist:{x:" + String(twist_msg_.linear_x) + ",y:" + String(twist_msg_.linear_y) + ",z:" + String(twist_msg_.angular_z) + "}\n").c_str());
        //         twistCallback_(&twist_msg_,"ConnectorWeb");
        // }   
        if (ConnectorWeb::jointCallback_ != NULL) {
            for(int i=0; i<NR_OF_JOINTS; i++){
                ConnectorWeb::jointCallback_(&joint_state_[i],"ConnectorWeb"); 
            }
        }
        if (isSyslog) syslog(LOG_DEBUG, "--- END listenAgent ---\n");
    );


    return true;
}

void ConnectorWeb::setTwist(double linear_x, double linear_y, double angular_z){
    twist_msg_.linear_x = linear_x;
    twist_msg_.linear_y = linear_y;
    twist_msg_.angular_z = angular_z;
}
float ConnectorWeb::getTwistX(){
    return twist_msg_.linear_x;
}
float ConnectorWeb::getTwistY(){
    return twist_msg_.linear_y;
}
float ConnectorWeb::getTwistZ(){
    return twist_msg_.angular_z;
}

void ConnectorWeb::setImu(float ax, float ay, float az, float gx, float gy, float gz, float qx, float qy, float qz, float qw){
    struct timespec time_stamp = getTime();

    //imu_msg.header.stamp.sec = time_stamp.tv_sec;
    //imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

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

    // #ifdef ENABLE_MICRO_ROS
    // imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
    // #endif
}

void ConnectorWeb::setMag(float x, float y, float z){
    mag_msg.magnetic_field.x = x;
    mag_msg.magnetic_field.y = y;
    mag_msg.magnetic_field.z = z;
} 

void ConnectorWeb::setOdometry(Odometry::Odometry_data pOdometry_data){
    struct timespec time_stamp = getTime();

    odom_msg= pOdometry_data;
    // odom_msg.header.stamp.sec = time_stamp.tv_sec;
    // odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    // //robot's position in x,y, and z
    // odom_msg.pose.pose.position.x = pOdometry_data.pose_pose_position_x;
    // odom_msg.pose.pose.position.y = pOdometry_data.pose_pose_position_y;
    // odom_msg.pose.pose.position.z = pOdometry_data.pose_pose_position_z;

    // //robot's heading in quaternion
    // odom_msg.pose.pose.orientation.x = pOdometry_data.pose_pose_orientation_x;
    // odom_msg.pose.pose.orientation.y = pOdometry_data.pose_pose_orientation_y;
    // odom_msg.pose.pose.orientation.z = pOdometry_data.pose_pose_orientation_z;
    // odom_msg.pose.pose.orientation.w = pOdometry_data.pose_pose_orientation_w;

    // odom_msg.pose.covariance[0]=pOdometry_data.pose_covariance[0];
    // odom_msg.pose.covariance[7]=pOdometry_data.pose_covariance[7];
    // odom_msg.pose.covariance[14]=pOdometry_data.pose_covariance[14];
    // odom_msg.pose.covariance[21]=pOdometry_data.pose_covariance[21];
    // odom_msg.pose.covariance[28]=pOdometry_data.pose_covariance[28];
    // odom_msg.pose.covariance[35]=pOdometry_data.pose_covariance[35];

    // //linear speed from encoders
    // odom_msg.twist.twist.linear.x = pOdometry_data.twist_twist_linear_x;
    // odom_msg.twist.twist.linear.y=pOdometry_data.twist_twist_linear_y;
    // odom_msg.twist.twist.linear.z=pOdometry_data.twist_twist_linear_z;

    // //angular speed from encoders
    // odom_msg.twist.twist.angular.x = pOdometry_data.twist_twist_angular_x;
    // odom_msg.twist.twist.angular.y=pOdometry_data.twist_twist_angular_y;
    // odom_msg.twist.twist.angular.z=pOdometry_data.twist_twist_angular_z;

    // odom_msg.twist.covariance[0]=pOdometry_data.twist_covariance[0];
    // odom_msg.twist.covariance[7]=pOdometry_data.twist_covariance[7];
    // odom_msg.twist.covariance[14]=pOdometry_data.twist_covariance[14];
    // odom_msg.twist.covariance[21]=pOdometry_data.twist_covariance[21];
    // odom_msg.twist.covariance[28]=pOdometry_data.twist_covariance[28];
    // odom_msg.twist.covariance[35]=pOdometry_data.twist_covariance[35];
}

void ConnectorWeb::setBattery(Battery::Battery_t pBattery_msg){

    struct timespec time_stamp = getTime();
    // battery_msg.header.stamp.sec = time_stamp.tv_sec;
    // battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    battery_msg.voltage = pBattery_msg.voltage;
    battery_msg.current = pBattery_msg.current;
    battery_msg.percentage = pBattery_msg.percentage;
    battery_msg.capacity = pBattery_msg.capacity;
    battery_msg.design_capacity = pBattery_msg.design_capacity;
    battery_msg.present = pBattery_msg.present;
}

void ConnectorWeb::setRange(float range){
    range_msg.range = range;
}
void ConnectorWeb::setRange(float range, float min_range, float max_range){
    struct timespec time_stamp = getTime();
    
    // range_msg.header.stamp.sec = time_stamp.tv_sec;
    // range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    range_msg.range = range;
    range_msg.min_range = min_range;
    range_msg.max_range = max_range;
}

void ConnectorWeb::setJointStateList(Connector::joint_state_t* pJointStateList){

    isPublishJointState=false;
    isPublishReqState=false;
    // Copy the contents of pJointStateList into joint_state_
    for(int i=0; i<NR_OF_JOINTS; i++){
        joint_state_[i] = pJointStateList[i];
    }

    // int listSize=sizeof(pJointStateList)/sizeof(pJointStateList[0]);
    // joint_state_msg.name.size = joint_state_msg.position.size = joint_state_msg.velocity.size = listSize;
    //  for(int i=0; i<listSize; i++){
    //     joint_state_[i].velocity = pJointStateList[i].velocity;
    //     joint_state_[i].position = pJointStateList[i].position;
    //     joint_state_[i].effort = pJointStateList[i].effort;
    //     joint_state_[i].velocity_requested = pJointStateList[i].velocity_requested;
    //     joint_state_[i].position_requested = pJointStateList[i].position_requested;
    //     joint_state_[i].effort_requested = pJointStateList[i].effort_requested;
    //     joint_state_[i].name = pJointStateList[i].name;
    //     if (isSyslog) syslog(LOG_INFO, ("   ConnectorWeb::setJointStateList, joint_state_["+String(i)+"].velocity=" + String(joint_state_[i].velocity) +"\n").c_str());
    //  }
}

void ConnectorWeb::publishWifi(DeviceWifi::DeviceWifi_t* pWifiData){
    deviceWifi_.id = pWifiData->id;
    deviceWifi_.name = pWifiData->name;
    deviceWifi_.mode = pWifiData->mode;
    deviceWifi_.ssid = pWifiData->ssid;
    deviceWifi_.ipAddress = pWifiData->ipAddress;
    deviceWifi_.macAddress = pWifiData->macAddress;
    deviceWifi_.rssi = pWifiData->rssi;

    if (isSyslog) syslog(LOG_DEBUG, ("   Wifi: ssid=" + deviceWifi_.ssid + ", ssid=" + deviceWifi_.ssid + ", rssi=" + String(deviceWifi_.rssi) + "dBm\n").c_str());
}

void ConnectorWeb::publishJoint(Connector::joint_state_t* pJointStateList){
    setJointStateList(pJointStateList);


    //if(isPublishJointState||isPublishReqState){
        // if (isSyslog) syslog(LOG_DEBUG, "   Joints: \n");
        // for(int i=0; i<NR_OF_JOINTS; i++){
        //     if ( pJointStateList[i].name != ""){
        //         if (isSyslog) syslog(LOG_DEBUG, ("      "+ String(pJointStateList[i].name) +": velocity="+String(pJointStateList[i].velocity) +",velocity_requested="+String(pJointStateList[i].velocity_requested) +"\n").c_str());
        //     }
        // }
    //}
}

void ConnectorWeb::publishRange(Range::Range_t pRange){
    setRange(pRange.range, pRange.min_range, pRange.max_range);

    if (isSyslog) syslog(LOG_DEBUG, ("   Range: range=" + String(range_msg.range) +"m\n").c_str());
}

bool ConnectorWeb::syncTime(){
    return true;
}

struct timespec ConnectorWeb::getTime()
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

void ConnectorWeb::publishImu(IMUInterface::Imu_t pImu_msg){

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

    if (isSyslog) syslog(LOG_DEBUG, ("   Imu: (x,y,z) lin.accel   =(" + String(imu_msg.linear_acceleration.x) +","+String(imu_msg.linear_acceleration.y)+","+String(imu_msg.linear_acceleration.z)+")\n").c_str());
    if (isSyslog) syslog(LOG_DEBUG, ("                ang.velo    =(" + String(imu_msg.angular_velocity.x) +","+String(imu_msg.angular_velocity.y)+","+String(imu_msg.angular_velocity.z)+")\n").c_str());
    if (isSyslog) syslog(LOG_DEBUG, ("                orientation =(" + String(imu_msg.orientation.x) +","+String(imu_msg.orientation.y)+","+String(imu_msg.orientation.z)+")\n").c_str());
}

void ConnectorWeb::publishMag(MAGInterface::Mag_t pMag_msg){
    setMag(pMag_msg.magnetic_field.x, 
        pMag_msg.magnetic_field.y, 
        pMag_msg.magnetic_field.z);

    struct timespec time_stamp = getTime();
    // mag_msg.header.stamp.sec = time_stamp.tv_sec;
    // mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    if (isSyslog) syslog(LOG_INFO, ("   Mag: (x,y,z) mag.field   =(" + String(mag_msg.magnetic_field.x) +","+String(mag_msg.magnetic_field.y)+","+String(mag_msg.magnetic_field.z)+")\n").c_str());
}

void ConnectorWeb::publishOdom(Odometry::Odometry_data pOdometry_data){
    setOdometry(pOdometry_data);

    struct timespec time_stamp = getTime();
    // odom_msg.header.stamp.sec = time_stamp.tv_sec;
    // odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    if (isSyslog) syslog(LOG_DEBUG, ("   Odom: (x,y,z) twist.linear=(" + String(odom_msg.twist_twist_linear_x) +","+String(odom_msg.twist_twist_linear_y)+","+String(odom_msg.twist_twist_linear_z)+")\n").c_str());
}

void ConnectorWeb::publishBattery(Battery::Battery_t pBattery_msg){
    setBattery(pBattery_msg);

    if (isSyslog) syslog(LOG_DEBUG, ("   Battery: voltage=" + String(battery_msg.voltage) +"V , current="+String(battery_msg.current)+"A ,percentage="+String(battery_msg.percentage)+"%\n").c_str());
}