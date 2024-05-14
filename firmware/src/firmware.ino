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
//#define ENABLE_PARAMETER

#include <Wire.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#ifdef ENABLE_PARAMETER
//   #include <micro_ros_arduino.h>
   #include <micro_ros_utilities/string_utilities.h>
   #include <micro_ros_utilities/type_utilities.h>
#endif

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rcl_interfaces/msg/log.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/joint_state.h>

#include "ArduinoLog.h"
#include "config.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
//#define ENCODER_USE_INTERRUPTS
//#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "motor.h"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop(temp_rc);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

#define NR_OF_JOINTS 4
#define SMOOTHING_CONST 0.95                // Coefficient for smoothing: smooth_pwm += (pwm - smooth_pwm)*SMOOTHING_CONST

rcl_subscription_t twist_subscriber;
rcl_publisher_t    odom_publisher;
rcl_publisher_t    imu_publisher;
rcl_publisher_t    joint_state_publisher;
rcl_publisher_t    req_state_publisher;

nav_msgs__msg__Odometry odom_msg;
nav_msgs__msg__Odometry odom_msg_old;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Imu imu_msg_old;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__JointState joint_state_msg;
sensor_msgs__msg__JointState joint_state_msg_old;
sensor_msgs__msg__JointState req_state_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_init_options_t init_options;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rclc_parameter_server_t param_server;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state = WAITING_AGENT;

float joint_rpm[NR_OF_JOINTS];
float req_rpm[NR_OF_JOINTS];


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

Odometry odometry;
IMU imu;

const char * kp_name         = "kp";
const char * ki_name         = "ki";
const char * kd_name         = "kd";
const char * rpm_ratio_name  = "scale";
const char * ned_to_enu_name = "ned_to_enu";

double kp = K_P;
double ki = K_I;
double kd = K_D;
double rpm_ratio = MAX_RPM_RATIO;
bool ned_to_enu = NED_TO_ENU;

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
    // --- Init Serial communication
    Wire.begin();
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    flashLED(1,2000);
    delay(2000);

    // --- Init ROS Communication
    // Wait ROS Agent
    set_microros_serial_transports(Serial);
    do{
       EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
       flashLED(1,120);
       delay(2000);
    } while (state==WAITING_AGENT);

    // Create ROS Subscriptions
    state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
    if (state == WAITING_AGENT) {
       destroyEntities();
    }
    flashLED(4,120);
    delay(5000);

    // --- Init devices
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


#ifdef ENABLE_PARAMETER
bool paramCallback(const Parameter * old_param, const Parameter * new_param, void * context)
{
  // check if an existing parameter is updated
  if (old_param != NULL && new_param != NULL)
  {
    // set changed parameter value
    if(strcmp(new_param->name.data, rpm_ratio_name) == 0){ rpm_ratio = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, kp_name) == 0){ kp = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, ki_name) == 0){ ki = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, kd_name) == 0){ kd = new_param->value.double_value; }
    else if(strcmp(new_param->name.data, ned_to_enu_name) == 0){ ned_to_enu = new_param->value.bool_value; }

    // update PID constants, max RPM, LED brightness - use existing values for unchanged parameters
    //kinematics.setMaxRPM(rpm_ratio);
    motor1_pid.updateConstants(kp, ki, kd);
    motor2_pid.updateConstants(kp, ki, kd);
    motor3_pid.updateConstants(kp, ki, kd);
    motor4_pid.updateConstants(kp, ki, kd);

    return true;
  }
  else return false; // reject loading of new parameters or deletion of existing parameters
}
#endif

bool createEntities()
{
    //allocator = rcl_get_default_allocator();
    //create init_options
    //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    //RCCHECK(rclc_node_init_default(&node, "linorobot_base_node", "", &support));

    // create and initialize node
    allocator = rcl_get_default_allocator();
    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, (size_t)ROS_DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, NODE, NAMESPACE, &support));

    //create logging
    RCCHECK(rclc_publisher_init_default(&publisher_log, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log), "rosout"));
    isLogInit=true;

    log(1,"-- START ROS createEntities");
    log(1,"   Connected to ROS micro-ros agent");

    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"
    ));
    log(1,"   Connected to ROS topic odom/unfitered");

    // create IMU publisher
    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"
    ));
    log(1,"   Connected to ROS topic imu/data");

    // create combined measured joint state publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &joint_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"));
    log(1,"   Connected to ROS topic joint_states");


    // create combined required state publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &req_state_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "req_states"));
    log(1,"   Connected to ROS topic req_states");

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    ));
    log(1,"   Connected to ROS topic cmd_vel");

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback
    ));
    log(1,"   #00#");

#ifdef ENABLE_PARAMETER
    // create parameter server
    const rclc_parameter_options_t param_options = {
        .notify_changed_over_dds = true,
        .max_params = 5,
        .allow_undeclared_parameters = true,
        .low_mem_mode = true};

    log(1,"   #01#");
    // initialize parameter server
    RCCHECK(rclc_parameter_server_init_with_option(&param_server, &node, &param_options));
    log(1,"   #02#");
#endif

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
    log(1,"   #03#");

#ifdef ENABLE_PARAMETER
    RCCHECK(rclc_executor_add_parameter_server(
        &executor,
        &param_server,
        paramCallback));
    log(1,"   #04#");
#endif

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA
    ));
    log(1,"   #05#");

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    log(1,"   Motor check timer created (frequency=50hz, timeout=20ms)");

#ifdef ENABLE_PARAMETER
    // add parameters to the server
    RCCHECK(rclc_add_parameter(&param_server, kp_name, RCLC_PARAMETER_DOUBLE));
    log(1,"   #06#");
    RCCHECK(rclc_add_parameter(&param_server, ki_name, RCLC_PARAMETER_DOUBLE));
    RCCHECK(rclc_add_parameter(&param_server, kd_name, RCLC_PARAMETER_DOUBLE));
    RCCHECK(rclc_add_parameter(&param_server, rpm_ratio_name, RCLC_PARAMETER_DOUBLE));
    RCCHECK(rclc_add_parameter(&param_server, ned_to_enu_name, RCLC_PARAMETER_BOOL));
    log(1,"   #07#");
    
    // add parameter constraints
    RCCHECK(rclc_add_parameter_constraint_double(&param_server, rpm_ratio_name, 0.0, 1.0, 0.01));
    log(1,"   #08#");

    // set parameter default values
    RCCHECK(rclc_parameter_set_double(&param_server, kp_name, K_P));
    RCCHECK(rclc_parameter_set_double(&param_server, ki_name, K_I));
    RCCHECK(rclc_parameter_set_double(&param_server, kd_name, K_D));
    RCCHECK(rclc_parameter_set_double(&param_server, rpm_ratio_name, MAX_RPM_RATIO));
    RCCHECK(rclc_parameter_set_bool(&param_server, ned_to_enu_name, NED_TO_ENU));
    log(1,"   #09#");
#endif

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

    rclc_executor_fini(&executor);
    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&imu_publisher, &node);
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_publisher_fini(&req_state_publisher, &node);
    rcl_publisher_fini(&publisher_log, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
#ifdef ENABLE_PARAMETER
    rclc_parameter_server_fini(&param_server, &node);
#endif
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);
    rcl_init_options_fini(&init_options);
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
  joint_rpm[2] = joint_rpm[0];
  joint_rpm[3] = joint_rpm[1];
/*
  joint_rpm[2] = motor3_encoder.getRPM();
  joint_rpm[3] = motor4_encoder.getRPM();
*/
  // check if relevant twist fields are not all zero
  if(true ||twist_msg.linear.x != 0 || twist_msg.linear.y != 0 || twist_msg.angular.z != 0) {
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        //digitalWrite(LED_PIN, HIGH);
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
    long smooth_pwm[NR_OF_JOINTS];
    for(int i=0; i<NR_OF_JOINTS; i++)
    {
      //smooth_pwm[i] += (pwm_arr[i] - smooth_pwm[i])*SMOOTHING_CONST;
      //smooth_pwm[i] = pwm_arr[i];
      if (req_rpm[i]==0)
         smooth_pwm[i]=0;
      else 
         smooth_pwm[i] = pwm_arr[i];
    }

    //For debug: display encoder position from the last restart
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
    //motor1_controller.spin(constrain(smooth_pwm[0], PWM_MIN, PWM_MAX));
    //motor2_controller.spin(constrain(smooth_pwm[1], PWM_MIN, PWM_MAX));
    motor1_controller.spin(smooth_pwm[0]);
    motor2_controller.spin(smooth_pwm[1]);
    //motor3_controller.spin(constrain(smooth_pwm[2], PWM_MIN, PWM_MAX));
    //motor4_controller.spin(constrain(smooth_pwm[3], PWM_MIN, PWM_MAX));

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

bool isPublishJointState=false;
bool isPublishReqState=false;
bool isPublish=false;
int JointStateVelocity[NR_OF_JOINTS];
int JointStatePosition[NR_OF_JOINTS];
int ReqStateVelocity[NR_OF_JOINTS];
int ReqStatePosition[NR_OF_JOINTS];
void publishData()
{
    // get current time
    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    odom_msg = odometry.getData();

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_msg = imu.getData();

    // populate measured joint state timestamp
    joint_state_msg.header.stamp.sec = time_stamp.tv_sec;
    joint_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    // populate required joint state timestamp
    req_state_msg.header.stamp.sec = time_stamp.tv_sec;
    req_state_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    isPublishJointState=false;
    isPublishReqState=false;
    isPublish=false;
    // populate required and measured joint states: velocities (rad/s) and positions (rads, in range[-pi, pi])
    for(int i=0; i<NR_OF_JOINTS; i++) {
       joint_state_msg.velocity.data[i] = joint_rpm[i] * M_PI * 2 / 60; // rpm to rad/s
       joint_state_msg.position.data[i] += joint_state_msg.velocity.data[i] / UPDATE_FREQ;  // rad/s to rad
       if (joint_state_msg.position.data[i] > M_PI){ joint_state_msg.position.data[i] = -1.0 * M_PI; }
       if (joint_state_msg.position.data[i] < -1.0 * M_PI){ joint_state_msg.position.data[i] = M_PI; }
       if (joint_state_msg.velocity.data[i] != JointStateVelocity[i] || 
           joint_state_msg.position.data[i] != JointStatePosition[i]){
          isPublishJointState=true;
          JointStateVelocity[i] = joint_state_msg.velocity.data[i]; 
          JointStatePosition[i] = joint_state_msg.position.data[i];
       }

       req_state_msg.velocity.data[i] = req_rpm[i] * M_PI * 2 / 60; // rpm to rad/s
       req_state_msg.position.data[i] += req_state_msg.velocity.data[i] / UPDATE_FREQ;  // rad/s to rad
       if (req_state_msg.position.data[i] > M_PI){ req_state_msg.position.data[i] = -1.0 * M_PI; }
       if (req_state_msg.position.data[i] < -1.0 * M_PI){ req_state_msg.position.data[i] = M_PI; }
       if (req_state_msg.velocity.data[i] != ReqStateVelocity[i] || 
           req_state_msg.position.data[i] != ReqStatePosition[i]){
          isPublishReqState=true;
          ReqStateVelocity[i] = req_state_msg.velocity.data[i]; 
          ReqStatePosition[i] = req_state_msg.position.data[i];
       }
    }

    if (!imu.equals(imu_msg_old)){
       RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
       imu.copy(&imu_msg_old);
       isPublish=true;
    }
    if (!odometry.equals(odom_msg_old)){
       RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
       odometry.copy(&odom_msg_old);
       isPublish=true;
    }
    if (isPublishJointState){
       RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));
       isPublish=true;
    }
    if (isPublishReqState){
       RCSOFTCHECK(rcl_publish(&req_state_publisher, &req_state_msg, NULL));
       isPublish=true;
    }

    if (isPublish){
       digitalWrite(LED_PIN, HIGH);
    }
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

void rclErrorLoop(rcl_ret_t ret)
{
    while(true) {
        flashLED(4,1000);
        delay(4000);
        //log(1,"error rcl %d",(long)ret);
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

      // get current time
      struct timespec time_stamp = getTime();

      msgLog.stamp.sec = time_stamp.tv_sec;
      msgLog.stamp.nanosec = time_stamp.tv_nsec;

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

