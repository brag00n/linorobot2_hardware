// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#if (ARDUINO > 100)
   #include <Arduino.h>
#else
   #include <WProgram.h>
#endif

#define DISABLE_LOGGING

#include <Wire.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl_interfaces/msg/log.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "ArduinoLog.h"
#include "config.h"
//#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
//#define ENCODER_USE_INTERRUPTS
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "motor.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

#define NR_OF_JOINTS 4
#define SMOOTHING_CONST 0.95                // Coefficient for smoothing: smooth_pwm += (pwm - smooth_pwm)*SMOOTHING_CONST

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

float joint_rpm[NR_OF_JOINTS];
float req_rpm[NR_OF_JOINTS];

Odometry odometry;
IMU imu;
long pos_motor1_previous=0;
long pos_motor2_previous=0;

//rclcpp::Logger logger = rclcpp::get_logger("motor_agent");
rcl_publisher_t publisher_log;
bool isLogInit = false;
char logBuffer[] = "";
char _ftoaBuffer[100];

char *ftoa(char *a, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 char formatString[1024], *ptr;
 strncpy_P( formatString, ret, sizeof(formatString) ); // copy in from program mem
 return ret;
}

char *ftoa(char *a, double f){
  return ftoa(a, f, 5);
}

char *ftoa(double f){
  char buffer[1024];
  return ftoa(buffer, f);
}

char *ftoa(double f, int precision){
  return ftoa(_ftoaBuffer, f,precision);
}

void setup() 
{

    log(1,"=== START setup  ===");
    Wire.begin();
    pinMode(LED_PIN, OUTPUT);
    //RCLCPP_INFO(logger, "--- START setup ---");
    flashLED(1,120);
    delay(2000);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
       destroyEntities();
    }
    flashLED(2,120);
    delay(5000);

    Log.trace("Test1 log\n");

    log(1,"=== START setup  ===");
    log(1,"   --- START init IMU ---");
    while(!imu.init()) {
       log(1,"   ERROR: could not init IMU (Waiting on IMU fixed ...)");
       flashLED(3,120);
       delay(5000);
    }
    log(1,"   IMU connected");
    log(1,"   --- END init IMU ---");

    log(1,"   --- START init Motor ---");
    while(!motor1_controller.initialize()) {
       log(1,"   ERROR: could not init Motor1 (Waiting on Motor fixed ...)");
       flashLED(3,120);
       delay(5000);
    }
    log(1,"   Motor1 connected");
    while(!motor2_controller.initialize()) {
       log(1,"   ERROR: could not init Motor2 (Waiting on Motor fixed ...)");
       flashLED(3,120);
       delay(5000);
    }
    log(1,"   Motor2 connected");
    while(!motor3_controller.initialize()) {
       log(1,"   ERROR: could not init Motor3 (Waiting on Motor fixed ...)");
       flashLED(3,120);
       delay(5000);
    }
    log(1,"   Motor3 connected");
    while(!motor4_controller.initialize()) {
       log(1,"   ERROR: could not init Motor4 (Waiting on Motor fixed ...)");
       flashLED(3,120);
       delay(5000);
    }
    log(1,"   Motor4 connected");
    log(1,"   --- END init Motor ---");
    log(1,"=== END setup  ===");
}

void loop() {
    digitalWrite(LED_PIN, LOW);
    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }else
               log(1,"   Reconnected to micro-ros agent");
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
            if (state == AGENT_CONNECTED) 
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
   }

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
       publishData();
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));
    //create logging
    RCCHECK(rclc_publisher_init_default(&publisher_log, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log), "rosout"));
    isLogInit=true;

    log(1,"-- START ROS createEntities");
    log(1,"   Connected to micro-ros agent");

    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    log(1,"   Connected to topic odom/unfitered");

    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));
    log(1,"   Connected to topic imu/data");

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    log(1,"   Connected to topic cmd_vel");
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    log(1,"   Motor check timer created (frequency=50hz, timeout=20ms)");
    log(1,"-- END ROS createEntities");

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&publisher_log, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    isLogInit=false;

    digitalWrite(LED_PIN, HIGH);
    
    return true;
}

void fullStop() {
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
    log(1,"   Full STOP executed");
}

void moveBase()
{

  // get measured RPM for each motor
  joint_rpm[0] = motor1_encoder.getRPM();
  joint_rpm[1] = motor2_encoder.getRPM();
  joint_rpm[2] = motor3_encoder.getRPM();
  joint_rpm[3] = motor4_encoder.getRPM();

  // check if relevant twist fields are not all zero
  if(true ||twist_msg.linear.x != 0 || twist_msg.linear.y != 0 || twist_msg.angular.z != 0) {
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }

    //twist_msg.linear.x = 0.005;

    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm request_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    req_rpm[0] = request_rpm.motor1;
    req_rpm[1] = request_rpm.motor2;
    req_rpm[2] = request_rpm.motor3;
    req_rpm[3] = request_rpm.motor4;

    // compute expected PWM for each motor
    float pwm_arr[NR_OF_JOINTS];
    pwm_arr[0] = motor1_pid.compute(req_rpm[0], joint_rpm[0]);
    pwm_arr[1] = motor2_pid.compute(req_rpm[1], joint_rpm[1]);
    pwm_arr[2] = motor3_pid.compute(req_rpm[2], joint_rpm[2]);
    pwm_arr[3] = motor4_pid.compute(req_rpm[3], joint_rpm[3]);

    // smoothing filter
    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    float smooth_pwm[NR_OF_JOINTS];
    for(int i=0; i<NR_OF_JOINTS; i++)
    {
      smooth_pwm[i] += (pwm_arr[i] - smooth_pwm[i])*SMOOTHING_CONST;
    }

    //For debug: display encoder position from the last resitart
    long pos_motor1=motor1_encoder.read();
    long pos_motor2=motor2_encoder.read();
    pos_motor1_previous=pos_motor1;   
    pos_motor2_previous=pos_motor2;   

    char buffer[200];
    if (joint_rpm[0]!=0 || joint_rpm[1]!=0 || pos_motor1!=pos_motor1_previous ||pos_motor2!=pos_motor2_previous){
       sprintf(buffer, "[POS1:%6d, RPM1:%6d, REQ_RPM1:%6d, PWM1_PID:%6d] [POS2:%6d, RPM2:%6d, REQ_RPM2:%6d, PWM2_PID:%6d]",
          pos_motor1,(long) joint_rpm[0],(long) round(request_rpm.motor1),(long) round(smooth_pwm[0]),
          pos_motor2,(long) joint_rpm[1],(long) round(request_rpm.motor2),(long) round(smooth_pwm[1]));
       log(1,buffer);
    }

    // spin motors according to smoothed pwm
    motor1_controller.spin(constrain(smooth_pwm[0], PWM_MIN, PWM_MAX));
    motor2_controller.spin(constrain(smooth_pwm[1], PWM_MIN, PWM_MAX));
    motor3_controller.spin(constrain(smooth_pwm[2], PWM_MIN, PWM_MAX));
    motor4_controller.spin(constrain(smooth_pwm[3], PWM_MIN, PWM_MAX));
  } else 
   stop();

    Kinematics::velocities current_vel = kinematics.getVelocities(
        joint_rpm[0], 
        joint_rpm[1], 
        joint_rpm[0], 
        joint_rpm[1]
    );

  // update odometry
    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
}

void stop()
{

  // set twist to 0
  twist_msg = {0.0};

  // brake all motors
  motor1_controller.brake();
  motor2_controller.brake();
  motor3_controller.brake();
  motor4_controller.brake();

  // reset all PID controllers
  motor1_pid.resetAll();
  motor2_pid.resetAll();
  motor3_pid.resetAll();
  motor4_pid.resetAll();

  // set required joint states to 0
  for(int i=0; i<NR_OF_JOINTS; i++)
  {
    req_rpm[i] = 0.0;
  }

}

void logEncoder(){

  float joint_rpm[NR_OF_JOINTS];
   //log(1,"logEncoder");
    Log.trace("Test1 log\n");
    // get the current speed of each motor
   joint_rpm[0] = motor1_encoder.getRPM();
   joint_rpm[1] = motor2_encoder.getRPM();
   int current_rpm3 = 0;
   int current_rpm4 = 0;

   char buffer[200];
   if (joint_rpm[0]!=0 || joint_rpm[1]!=0){
    sprintf(buffer, "RPM1:%6d, RPM2:%6d, RPM3:%6d, RPM4:%6d\n",joint_rpm[0],joint_rpm[1],current_rpm3,current_rpm4);
    log(1,buffer);
   }
}

void publishData()
{
        odom_msg = odometry.getData();
    imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop() 
{
    while(true) {
        flashLED(4,1000);
        delay(4000);
    }
}

void flashLED(int n_times,int pause)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(pause);
        digitalWrite(LED_PIN, LOW);
        delay(pause);
    }
}
char* string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
}
void log(int pLevel, char *pMessage){

   if (isLogInit){ 
      rcl_interfaces__msg__Log msgLog;
      msgLog.level = rcl_interfaces__msg__Log__INFO;
      msgLog.name.data = "Teensy01";
      msgLog.name.size = strlen(msgLog.name.data);
//      msgLog.msg.data = pMessage;
//      msgLog.msg.size = strlen(msgLog.msg.data);
      msgLog.file.data = "";
      msgLog.file.size = strlen(msgLog.file.data);
      msgLog.function.data = "";
      msgLog.function.size = strlen(msgLog.function.data);
      msgLog.line = NULL;
/*      if (strlen(logBuffer)>0){
         msgLog.msg.data = logBuffer;
         msgLog.msg.size = strlen(msgLog.msg.data);
         RCSOFTCHECK(rcl_publish(&publisher_log, &msgLog, NULL));
         strcpy( logBuffer, "");
      }*/   
      msgLog.msg.data = pMessage;
      msgLog.msg.size = strlen(msgLog.msg.data);
      RCSOFTCHECK(rcl_publish(&publisher_log, &msgLog, NULL));
   }
//   else
//      strcpy( logBuffer, pMessage);
}

