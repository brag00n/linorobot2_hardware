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

#include "lino_base_config.h"

// --- --- --- i2c Settings --- --- ---

#define S_SCL   33
#define S_SDA   32

// --- --- --- --- --- --- --- --- ---

#include <Arduino.h>
#include <i2cdetect.h>

#include "MySyslog.h"
#include "led.h"
#include "motor.h"
#include "kinematics.h"
#include "odometry.h"
#include "pid.h"

#include "imu.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "battery.h"
#include "range.h"
#include "lidar.h"
//#include "wifis.h"
#include "ota.h"
#include "pwm.h"
#include "oled_ctrl.h"
#include "ros.hpp"

#ifdef WDT_TIMEOUT
#include <esp_task_wdt.h>
#endif

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
float prev_voltage;
float prev_current;
ConnectorMicroROS::joint_state_t jointState[8];


enum states{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
//Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
//Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

float previous_rpm1= 0;
float previous_rpm2= 0;
float previous_rpm3= 0;
float previous_rpm4= 0;

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
MAG mag;
Battery battery;
Range range;
ConnectorMicroROS connector;

#ifdef ENABLE_MICRO_ROS
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis();} \
    if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
  } while (0)
#else
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = millis();} \
    if (millis() - init > MS) { X; init = millis();} \
  } while (0)
#endif

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        setLed(HIGH);
        delay(150);
        setLed(LOW);
        delay(150);
    }
    delay(1000);
}

void fullStop()
{
    connector.setTwist(0, 0, 0);

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

void rclErrorLoop()
{
    while(true)
    {
        flashLED(2); // flash 2 times
        runOta();
    }
}

void moveBase()
{
    if (isSyslog) syslog(LOG_DEBUG, "moveBase\n");
    // brake if there's no command received, or when it's only the first command sent

    //connector.setTwist(0.1, 0, 0.1);
    if(((millis() - prev_cmd_time) >= 200))
    {
        connector.setTwist(0, 0, 0);
        setLed(HIGH);
    }
    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        connector.getTwistX(),
        connector.getTwistY(),
        connector.getTwistZ()
    );

    // get the current speed of each motor
    float current_rpm1 = jointState[0].velocity = motor1_encoder.getRPM();
    float current_rpm2 = jointState[1].velocity = motor2_encoder.getRPM();
    //float current_rpm3 = motor3_encoder.getRPM();
    //float current_rpm4 = motor4_encoder.getRPM();
    float current_rpm3 = jointState[2].velocity= current_rpm1;
    float current_rpm4 = jointState[3].velocity= current_rpm2;

    jointState[0].velocity_requested= req_rpm.motor1;
    jointState[1].velocity_requested= req_rpm.motor2;
    jointState[2].velocity_requested= req_rpm.motor3;
    jointState[3].velocity_requested= req_rpm.motor4;

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));

    if ((current_rpm1 != previous_rpm1) || (current_rpm2 != previous_rpm2) || (current_rpm3 != previous_rpm3) || (current_rpm4 != previous_rpm4)){
        if (isSyslog) syslog(LOG_INFO, ("   RPM01("+String(req_rpm.motor1)+")=" + String(current_rpm1) + ",PWM01="+String(motor1_pid.compute(req_rpm.motor1, current_rpm1))+", RPM02("+String(req_rpm.motor2)+")="+ String(current_rpm2)+",PWM02="+String(motor2_pid.compute(req_rpm.motor2, current_rpm2))+"\n").c_str());
    }

    previous_rpm1= current_rpm1;
    previous_rpm2= current_rpm2;
    previous_rpm3= current_rpm3;
    previous_rpm4= current_rpm4;

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1,
        current_rpm2,
        current_rpm3,
        current_rpm4
    );

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

void publishData()
{
    if (isSyslog) syslog(LOG_DEBUG, "publishData\n");
    static unsigned skip_dip = 0;
    Odometry::Odometry_data odom_msg = odometry.getData();
    IMUInterface::Imu_t imu_msg = imu.readIMU().getData();

#ifdef USE_FAKE_IMU
    imu_msg.angular_velocity.z = odom_msg.twist_twist_angular_z;
#endif

    MAGInterface::Mag_t mag_msg = mag.readMAG().getData();

#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif

#ifndef USE_FAKE_MAG
 #ifdef ENABLE_MICRO_ROS
    struct timespec time_stamp = connector.getTime();
    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
 #endif
#endif

    connector.publishImu(imu_msg);

#ifndef USE_FAKE_MAG
    connector.publishMag(mag_msg);
#endif

    connector.publishOdom(odom_msg);


    EXECUTE_EVERY_N_MS(BATTERY_TIMER, {
        connector.publishBattery(battery.readBattery().getData());

        prev_voltage = battery.getVoltage() * 0.01 + prev_voltage * 0.99;
        prev_current = battery.getCurrent() * 0.01 + prev_current * 0.99;
        if (isSyslog) syslog(LOG_INFO, ("   battery voltage " + String(prev_voltage) + "V\n").c_str());
        screenLine_1 = "started";
        screenLine_2 = "batt: " + String(prev_voltage)+"V, "+ String(prev_current)+"A";
        screenLine_3 = "";
        oled_update();
    });


//#ifdef ECHO_PIN
    EXECUTE_EVERY_N_MS(RANGE_TIMER, {
        connector.publishRange(range.getRange()); 
    });
//#endif

EXECUTE_EVERY_N_MS(RANGE_TIMER, {
    connector.publishJoint(jointState); 
});

}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
       if (isSyslog) syslog(LOG_DEBUG, "controlCallback\n");
       moveBase();
       publishData();
    }

}

void twistCallback(const void * msgin){
    //if (isSyslog) syslog(LOG_DEBUG, "twistCallback\n");
    setLed(!getLed());
    prev_cmd_time = millis();
}


void jointCallback(const void *msgin)
{
    #ifdef JOINT_STATE_SUBSCRIBER
    if (isSyslog) syslog(LOG_DEBUG, "jointCallback\n");
    #endif
}

bool createEntities()
{
    bool initOK=connector.initAgent(controlCallback,twistCallback,jointCallback);
    // synchronize time with the agent
    bool timeOK=connector.syncTime();
    setLed(HIGH);

    if (isSyslog) syslog(LOG_DEBUG, "agent available\n");
    return (initOK && timeOK);
}

bool destroyEntities()
{
    if (isSyslog) syslog(LOG_DEBUG, "destroy agent entities");

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
    setLed(HIGH);

    return true;
}

void setup()
{
    state = WAITING_AGENT;
#ifdef ESP32
    Serial.setRxBufferSize(1024);
#endif
    Serial.begin(BAUDRATE);
    initLed();

#ifdef BOARD_INIT // board specific setup, must include Wire.begin
    BOARD_INIT
#else
    Wire.begin(S_SDA, S_SCL);
#endif

    while(!Serial) {}

    if (isSyslog) syslog(LOG_INFO, "\n=========START setup() =========\n");

    if (isSyslog) syslog(LOG_INFO, "--- init_oled\n");
    init_oled();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    screenLine_0 = "BAMBOO v0.01 OCO";
    screenLine_1 = "Setting up...";
    screenLine_2 = "";
    screenLine_3 = "";
    oled_update();

#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
#endif

    if (isSyslog) syslog(LOG_INFO, "--- initWifis\n");
    initWifis();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    if (isSyslog) syslog(LOG_INFO, "--- initOta\n");
    initOta();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    if (isSyslog) syslog(LOG_INFO, "--- i2cdetect\n");
    i2cdetect();  // default range from 0x03 to 0x77
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    if (isSyslog) syslog(LOG_INFO, "--- initPwm\n");
    initPwm();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");


    if (isSyslog) syslog(LOG_INFO, "--- init motors\n");
    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    if (isSyslog) syslog(LOG_INFO, "--- init IMU\n");
    bool imu_ok = imu.init();
    if (!imu_ok) // take IMU failure as fatal
    {
        Serial.println("   IMU init failed");
        if (isSyslog) syslog(LOG_DEBUG, "   IMU init failed\n");
        while (1)
        {
            flashLED(3); // flash 3 times
            runWifis();
            runOta();
        }
    }
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    if (isSyslog) syslog(LOG_INFO, "--- init Mag\n");
    bool mag_ok = mag.init();
    if (!mag_ok) // take IMU failure as fatal
    {
        Serial.println("MAG init failed");
        if (isSyslog) syslog(LOG_DEBUG, "%s MAG init failed %lu", __FUNCTION__, millis());
        while (1)
        {
            flashLED(4); // flash 4 times
            runWifis();
            runOta();
        }
    }
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");


    if (isSyslog) syslog(LOG_INFO, "--- init Battery\n");
    battery.initBattery();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    if (isSyslog) syslog(LOG_INFO, "--- init ultrasonic sensor\n");
    range.initRange();
    jointState[0].name = "joint_motor1";
    jointState[1].name = "joint_motor2";
    jointState[2].name = "joint_motor3";
    jointState[3].name = "joint_motor4";
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");


    if (isSyslog) syslog(LOG_INFO, "--- init lidar\n");
    initLidar(); // after wifi connected
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

#ifdef JOINT_STATE_SUBSCRIBER
    // allocate dynamic msg memory
    static micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 20;
    conf.max_ros2_type_sequence_capacity = 10;
    conf.max_basic_type_sequence_capacity = 10;
    bool success = micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        &joint_msg,
        conf
    );
    if (isSyslog) syslog(LOG_DEBUG, "%s %lu allocate msg %d", __FUNCTION__, millis(), success);
#endif

#ifdef ENABLE_MICRO_ROS
    if (isSyslog) syslog(LOG_DEBUG, "--- init micro-ros\n");
    connector.initMicroRos();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif
#endif


#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif

    if (isSyslog) syslog(LOG_DEBUG, "--- get Battery voltage\n");
    battery.readBattery();
    prev_voltage = battery.getVoltage();
    prev_current = battery.getCurrent();
    if (isSyslog) syslog(LOG_DEBUG, ("   battery: " + String(prev_voltage) + "V, "+ String(prev_current) + "A\n").c_str());
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");

    screenLine_1 = "Ready";
    screenLine_2 = "batt: " + String(prev_voltage)+"V, "+ String(prev_current)+"A";
    screenLine_3 = "";
    oled_update();

    if (isSyslog) syslog(LOG_DEBUG, "=========END setup() =========\n");
}

void loop() {
    battery.readBattery();
    prev_voltage = battery.getVoltage() * 0.01 + prev_voltage * 0.99;
    prev_current = battery.getCurrent() * 0.01 + prev_current * 0.99;
    switch (state)
    {
        case WAITING_AGENT:         
            EXECUTE_EVERY_N_MS(500, 
                if (isSyslog) syslog(LOG_DEBUG, "check agent availability\n"); 
                state = (connector.pingAgent(100,1)) ? AGENT_AVAILABLE : WAITING_AGENT;
                screenLine_1 = "connecting to agent...";
                oled_update();
            );
            break;
        case AGENT_AVAILABLE:
            state = (createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == AGENT_CONNECTED) {
                screenLine_1 = "Ready";
                oled_update();
            }
            if (state == WAITING_AGENT) destroyEntities();
            break;
        case AGENT_CONNECTED:
            EXECUTE_EVERY_N_MS(200, 
                state = (connector.pingAgent(100,1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            );
            if (state == AGENT_CONNECTED){
                connector.listenAgent(100);
            }
            break;
        case AGENT_DISCONNECTED:
            if (isSyslog) syslog(LOG_DEBUG, "agent disconnected\n");

            screenLine_1 = "Disconnected to agent";
            oled_update();

            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    runWifis();
    runOta();
#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
#ifdef BOARD_LOOP // board specific loop
    BOARD_LOOP
#endif
}
