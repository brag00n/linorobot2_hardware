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


//#define ENABLE_DEBUG_I2C
#define ENABLE_DEVICE_OLED
#define ENABLE_DEVICE_WIFI
#define ENABLE_OTA
#define ENABLE_DEVICE_SERVO_MOTOR
#define ENABLE_DEVICE_BATTERY
#define ENABLE_DEVICE_LIDAR
#define ENABLE_DEVICE_IMU
#define ENABLE_DEVICE_MAG
#define ENABLE_DEVICE_ULTRASONIC
#define ENABLE_DEVICE_ENCODER
#define ENABLE_DEVICE_MOTOR
#define ENABLE_DEVICE_PID
#ifdef ENABLE_DEVICE_WIFI
    #define ENABLE_CONNECTOR_WEB
#endif // ENABLE_DEVICE_WIFI
//#define ENABLE_MICRO_ROS
#define ENABLE_CONNECTOR_ROS

#include "config.h"

// --- --- --- i2c Settings --- --- ---

#define S_SCL   33
#define S_SDA   32

// --- --- --- --- --- --- --- --- ---

#include <Arduino.h>
#include <Wire.h>
#include "led.h"

#include "MySyslog.h"

#ifdef ENABLE_DEBUG_I2C 
   #include <i2cdetect.h> 
#endif

#ifdef ENABLE_DEVICE_OLED
    #include "oled_ctrl.h"
#endif

#ifdef ENABLE_DEVICE_BATTERY
    #include "battery.h"
#endif

#ifdef ENABLE_DEVICE_MOTOR
    #include "motor.h"
#endif // ENABLE_DEVICE_MOTOR

#include "kinematics.h"

#ifdef ENABLE_DEVICE_ENCODER
    #include "odometry.h"
#endif // ENABLE_DEVICE_ENCODER

#ifdef ENABLE_DEVICE_PID
    #include "pid.h"
#endif // ENABLE_DEVICE_PID

#ifdef ENABLE_DEVICE_IMU
    #include "imu_interface.h"
    #include "imu.h"
#endif

#ifdef ENABLE_DEVICE_MAG
    #include "mag.h"
#endif //  ENABLE_DEVICE_MAG

#ifdef ENABLE_DEVICE_ENCODER
    #define ENCODER_USE_INTERRUPTS
    #define ENCODER_OPTIMIZE_INTERRUPTS
    #include "encoder.h"
#endif // ENABLE_DEVICE_ENCODER

#ifdef ENABLE_DEVICE_ULTRASONIC
#include "range.h"
#endif // def ENABLE_DEVICE_ULTRASONIC

#ifdef ENABLE_DEVICE_LIDAR
   #include "lidar.h"
#endif

#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID "<SSID>" // Replace with your WiFi SSID
#endif
#ifndef WIFI_STA_PASSWORD
#define WIFI_STA_PASSWORD "<PASSWORD>" // Replace with your WiFi password
#endif
#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "<SSID>" // Replace with your WiFi AP SSID
#endif
#ifndef WIFI_AP_PASSWORD
#define WIFI_AP_PASSWORD "<PASSWORD>" // Replace with your WiFi AP password
#endif

#ifdef ENABLE_OTA
#include "ota.h"
#endif // ENABLE_OTA

#ifdef ENABLE_DEVICE_SERVO_MOTOR
#include "pwm.h"
#endif // ENABLE_DEVICE_SERVO_MOTOR

#include "Connector.hpp"

#ifdef ENABLE_CONNECTOR_ROS
    #include "ConnectorMicroROS.hpp"
#endif // ENABLE_CONNECTOR_ROS

#ifdef ENABLE_CONNECTOR_WEB
    #include "ConnectorWeb.hpp"
#endif

//#define log_info(format, ...) if (isSyslog) syslog(LOG_INFO,"[%s:%u]: ",pathToFileName(__FILE__), __LINE__)

#ifdef ENABLE_DEVICE_ULTRASONIC
#ifndef RANGE_TIMER
#define RANGE_TIMER 100 // 10Hz
#endif
#endif // ENABLE_DEVICE_ULTRASONIC

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

Connector::joint_state_t jointState[8];


#ifdef ENABLE_DEVICE_ENCODER
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
//Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
//Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);
#endif // ENABLE_DEVICE_ENCODER

float previous_rpm1= 0;
float previous_rpm2= 0;
float previous_rpm3= 0;
float previous_rpm4= 0;

#ifdef ENABLE_DEVICE_MOTOR
Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);
#endif // ENABLE_DEVICE_MOTOR

#ifdef ENABLE_DEVICE_PID
PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
#endif // ENABLE_DEVICE_PID

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

#ifdef ENABLE_DEVICE_ENCODER
Odometry odometry;
#endif // ENABLE_DEVICE_ENCODER

#ifdef ENABLE_DEVICE_IMU
IMU imu;
#endif

#ifdef ENABLE_DEVICE_MAG
MAG mag;
#endif // ENABLE_DEVICE_MAG

#ifdef ENABLE_DEVICE_BATTERY
Battery battery;
#endif

#ifdef ENABLE_DEVICE_ULTRASONIC
Range range;
#endif // ENABLE_DEVICE_ULTRASONIC

#ifdef ENABLE_DEVICE_WIFI
DeviceWifi wifiDevice;
#endif

#ifdef ENABLE_CONNECTOR_ROS
ConnectorMicroROS  connectorROS;
#endif // ENABLE_CONNECTOR_ROS

#ifdef ENABLE_CONNECTOR_WEB
ConnectorWeb connectorWeb;
#endif // ENABLE_DEVICE_WIFI


#ifdef ENABLE_CONNECTOR_ROS
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = connectorROS.getMillis();} \
    if (connectorROS.getMillis() - init > MS) { X; init = connectorROS.getMillis();} \
  } while (0)
#else
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = millis();} \
    if (millis() - init > MS) { X; init = millis();} \
  } while (0)
#endif // ENABLE_CONNECTOR_ROS


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

void update_oled(const char *line0, const char *line1, const char *line2, const char *line3){
#ifdef ENABLE_DEVICE_OLED
    if (line0 != NULL) screenLine_0=line0;
    if (line1 != NULL) screenLine_1=line1;
    if (line2 != NULL) screenLine_2=line2;
    if (line3 != NULL) screenLine_3=line3;
    oled_update();
#endif
}

void fullStop()
{
#ifdef ENABLE_CONNECTOR_ROS
    connectorROS.setTwist(0, 0, 0);
#endif // ENABLE_CONNECTOR_ROS

#ifdef ENABLE_DEVICE_MOTOR
    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
#endif // ENABLE_DEVICE_MOTOR
}

#ifndef ERROR_LOOP_CALLBACK
void ErrorLoopCallback(int error_code)
{
    bool first_run = true;
    while(true)
    {
        flashLED(2); // flash 2 times
        if (first_run) {
            first_run = false;
            if (isSyslog) syslog(LOG_ERR, "ROS Error: %d\n", error_code);
#ifdef ENABLE_DEVICE_OLED
        update_oled(NULL, ("ROS Error: " + String(error_code)).c_str(), NULL, NULL);
#endif // ENABLE_DEVICE_OLED
        }

#if defined(ENABLE_DEVICE_WIFI) && defined(ENABLE_OTA)
        if (wifiDevice.isReady()) { runOta();  } // try to update code using OTA
        delay(100); // wait a bit before next iteration
#endif // ENABLE_DEVICE_WIFI
    }
}
#define ERROR_LOOP_CALLBACK
#endif // ERROR_LOOP_CALLBACK

void moveBase()
{
    if (isSyslog) syslog(LOG_DEBUG, "moveBase\n");
    // brake if there's no command received, or when it's only the first command sent

#ifdef ENABLE_CONNECTOR_ROS

    //connectorROS.setTwist(0, 0, 0);
    if (!(connectorROS.getTwistX() == 0 && connectorROS.getTwistY() == 0 && connectorROS.getTwistZ() == 0)) {
        if (isSyslog) syslog(LOG_DEBUG, ("    firmware::moveBase (on js cmd 1): Twist:{x:" + String(connectorROS.getTwistX()) + ",y:" + String(connectorROS.getTwistY()) + ",z:" + String(connectorROS.getTwistZ()) + "}\n").c_str());
    }

    if(((millis() - prev_cmd_time) >= 500) && connectorROS.getTwistX() != 0 && connectorROS.getTwistY() != 0 && connectorROS.getTwistZ() != 0)
    {
        connectorROS.setTwist(0, 0, 0);
        if (isSyslog) syslog(LOG_INFO, "    firmware::moveBase Stop (no activity)\n");
        setLed(HIGH);
    }

    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        connectorROS.getTwistX(),
        connectorROS.getTwistY(),
        connectorROS.getTwistZ()
    );

#ifdef ENABLE_DEVICE_ENCODER
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
#ifdef ENABLE_DEVICE_MOTOR
#ifdef ENABLE_DEVICE_PID
    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));
#else
    motor1_controller.spin(map(req_rpm.motor1, 0,MOTOR_MAX_RPM * MAX_RPM_RATIO,PWM_MIN, PWM_MAX));
    motor2_controller.spin(map(req_rpm.motor2, 0,MOTOR_MAX_RPM * MAX_RPM_RATIO,PWM_MIN, PWM_MAX));
    motor3_controller.spin(map(req_rpm.motor3, 0,MOTOR_MAX_RPM * MAX_RPM_RATIO,PWM_MIN, PWM_MAX));
    motor4_controller.spin(map(req_rpm.motor4, 0,MOTOR_MAX_RPM * MAX_RPM_RATIO,PWM_MIN, PWM_MAX));
#endif // ENABLE_DEVICE_PID
#endif // ENABLE_DEVICE_MOTOR

#ifdef ENABLE_DEVICE_PID
    if ((current_rpm1 != previous_rpm1) || (current_rpm2 != previous_rpm2) || (current_rpm3 != previous_rpm3) || (current_rpm4 != previous_rpm4)){
        if (isSyslog) syslog(LOG_DEBUG, ("   RPM01("+String(req_rpm.motor1)+")=" + String(current_rpm1) + ",PWM01="+String(motor1_pid.compute(req_rpm.motor1, current_rpm1))+", RPM02("+String(req_rpm.motor2)+")="+ String(current_rpm2)+",PWM02="+String(motor2_pid.compute(req_rpm.motor2, current_rpm2))+", Twist:{lin.x:" + String(connectorROS.getTwistX()) + ",lin.y:" + String(connectorROS.getTwistY()) + ",ang.z:" + String(connectorROS.getTwistZ()) + "}\n").c_str());
    }
#endif // ENABLE_DEVICE_PID

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
#endif // ENABLE_DEVICE_ENCODER
#endif // ENABLE_CONNECTOR_ROS

}

void publishData()
{
    if (isSyslog) syslog(LOG_DEBUG, "publishData\n");
    static unsigned skip_dip = 0;

#ifdef ENABLE_DEVICE_ENCODER
    Odometry::Odometry_data odom_msg = odometry.getData();
#ifdef ENABLE_CONNECTOR_ROS
    connectorROS.publishOdom(odom_msg);
#endif // ENABLE_CONNECTOR_ROS

#ifdef ENABLE_CONNECTOR_WEB
    connectorWeb.publishOdom(odom_msg);
#endif
#endif // ENABLE_DEVICE_ENCODER

    // publish imu data
#ifdef ENABLE_DEVICE_IMU
    IMUInterface::Imu_t imu_msg = imu.readIMU().getData();
#ifdef USE_FAKE_IMU
#ifdef ENABLE_DEVICE_ENCODER
    imu_msg.angular_velocity.z = odom_msg.twist_twist_angular_z;
#endif // ENABLE_DEVICE_ENCODER
#endif // USE_FAKE_IMU

#ifdef ENABLE_CONNECTOR_ROS
    connectorROS.publishImu(imu_msg);
#endif // ENABLE_CONNECTOR_ROS

#ifdef ENABLE_CONNECTOR_WEB
    connectorWeb.publishImu(imu_msg);
#endif // ENABLE_CONNECTOR_WEB

#endif // ENABLE_DEVICE_IMU

    // publish Mag data
#ifdef ENABLE_DEVICE_MAG
    MAGInterface::Mag_t mag_msg = mag.readMAG().getData();
#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif
#ifndef USE_FAKE_MAG
  #ifdef ENABLE_CONNECTOR_ROS
    struct timespec time_stamp = connectorROS.getTime();
    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  #endif // ENABLE_CONNECTOR_ROS
#endif
#ifndef USE_FAKE_MAG

#ifdef ENABLE_CONNECTOR_ROS
    connectorROS.publishMag(mag_msg);
#endif // ENABLE_CONNECTOR_ROS

#ifdef ENABLE_CONNECTOR_WEB
    connectorWeb.publishMag(mag_msg);
#endif // ENABLE_CONNECTOR_WEB
#endif // ENABLE_DEVICE_MAG

#endif

#ifdef ENABLE_DEVICE_BATTERY
    EXECUTE_EVERY_N_MS(BATTERY_TIMER, {

        battery.readBattery();
#ifdef ENABLE_CONNECTOR_ROS
        connectorROS.publishBattery(battery.getData());
#endif // ENABLE_CONNECTOR_ROS
#ifdef ENABLE_CONNECTOR_WEB
        connectorWeb.publishBattery(battery.getData());
#endif // ENABLE_CONNECTOR_WEB
        prev_voltage = battery.getVoltage() * 0.01 + prev_voltage * 0.99;
        prev_current = battery.getCurrent();// * 0.01 + prev_current * 0.99;
        if (isSyslog) syslog(LOG_DEBUG, ("   battery voltage:" + String(prev_voltage) + "V, current:" + String(prev_current) + "A\n").c_str());

#ifdef ENABLE_DEVICE_OLED
        update_oled(NULL,
            "started", 
            ("batt: " + String(prev_voltage) + "V, " + String(prev_current) + "A").c_str(), 
            NULL
        );
#endif // ENABLE_DEVICE_OLED

#ifdef ENABLE_CONNECTOR_WEB
        wifiDevice.read();
        connectorWeb.publishWifi(wifiDevice.getData());
#endif // ENABLE_CONNECTOR_WEB

    });
#endif


#ifdef ENABLE_DEVICE_ULTRASONIC
//#ifdef ECHO_PIN
    EXECUTE_EVERY_N_MS(RANGE_TIMER, {
#ifdef ENABLE_CONNECTOR_ROS
        connectorROS.publishRange(range.getRange());
#endif // ENABLE_CONNECTOR_ROS
    });
//#endif
#endif // ENABLE_DEVICE_ULTRASONIC

//EXECUTE_EVERY_N_MS(RANGE_TIMER, {
#ifdef ENABLE_CONNECTOR_ROS
    connectorROS.publishJoint(jointState);
#endif // ENABLE_CONNECTOR_ROS

#ifdef ENABLE_CONNECTOR_WEB
    connectorWeb.publishJoint(jointState);
#endif // ENABLE_CONNECTOR_WEB 
//});

}

void controlCallback(Connector::timer_t * timer, int64_t last_call_time)
{
    //RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
       if (isSyslog) syslog(LOG_DEBUG, "controlCallback\n");
       moveBase();
       publishData();
    }

}

void twistCallback(const Connector::Twist_t* pTwist){
#ifdef ENABLE_CONNECTOR_ROS
    connectorROS.setTwist(pTwist->linear_x,pTwist->linear_y , pTwist->angular_z);
#endif // ENABLE_CONNECTOR_ROS

    if (isSyslog) syslog(LOG_DEBUG, ("   twistCallback: Twist:{x:" + String(pTwist->linear_x) + ",y:" + String(pTwist->linear_y) + ",z:" + String(pTwist->angular_z) + "}\n").c_str());
    setLed(!getLed());
    prev_cmd_time = millis();
}


void jointCallback(const void *msgin)
{
    #ifdef JOINT_STATE_SUBSCRIBER
    if (isSyslog) syslog(LOG_DEBUG, "jointCallback\n");
    #endif
}

void pidCallback(const Connector::Pid_t* pPid){
    if (isSyslog) syslog(LOG_INFO, ("   pidCallback: P:" + String(pPid->P) + ", I:" + String(pPid->I) + ", D:" + String(pPid->D) + "\n").c_str());
#ifdef ENABLE_DEVICE_PID
    motor1_pid.updateConstants(pPid->P, pPid->I, pPid->D);
    motor2_pid.updateConstants(pPid->P, pPid->I, pPid->D);
    motor3_pid.updateConstants(pPid->P, pPid->I, pPid->D);
    motor4_pid.updateConstants(pPid->P, pPid->I, pPid->D);
#endif // ENABLE_DEVICE_PID
}

bool createEntities()
{
bool initROSOK=true,timeOK=true;
#ifdef ENABLE_CONNECTOR_ROS
    initROSOK=connectorROS.initAgent(*controlCallback,*twistCallback,*jointCallback,NULL);
    timeOK=   connectorROS.syncTime(); // synchronize time with the agent
#endif // ENABLE_CONNECTOR_ROS
    setLed(HIGH);
    if (isSyslog) syslog(LOG_DEBUG, "agent available\n");
    return (initROSOK && timeOK);
}


void setup()
{

#ifdef ESP32
    Serial.setRxBufferSize(1024);
#endif
    Serial.begin(BAUDRATE);
    initLed();

#ifdef BOARD_INIT // board specific setup, must include Wire.begin
    BOARD_INIT
#else
    Wire.begin(S_SDA, S_SCL); // I2C configuration
#endif

    while(!Serial) {} // wait for serial port to connect. Needed for native USB port only

    if (isSyslog) syslog(LOG_INFO,"\n=========START setup() =========\n");

#ifdef ENABLE_DEVICE_OLED
    if (isSyslog) syslog(LOG_INFO, "--- init_oled\n");
    init_oled();
    update_oled("BAMBOO v0.01 OCO", "Setting up...", "", "");
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif

#ifdef WDT_TIMEOUT
    esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
    esp_task_wdt_add(NULL); //add current thread to WDT watch
#endif

#ifdef ENABLE_DEVICE_WIFI
    if (isSyslog) syslog(LOG_INFO, "--- init Wifi\n");
    update_oled(NULL, "Init Wifi", NULL, NULL);
    // initWifis();
    wifiDevice.setSta(WIFI_STA_SSID, WIFI_STA_PASSWORD); // set your wifi credentials here
    wifiDevice.setAp(WIFI_AP_SSID, WIFI_AP_PASSWORD); // set your AP credentials here
    wifiDevice.init("Wifi");
    update_oled(NULL, "Init Wifi", NULL, (wifiDevice.getData()->mode + ": " + wifiDevice.getData()->ipAddress).c_str());
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif // ENABLE_DEVICE_WIFI

#if defined(ENABLE_DEVICE_WIFI) && defined(ENABLE_OTA)
    if (wifiDevice.isReady()) { 
        if (isSyslog) syslog(LOG_INFO, "--- init OTA\n");
        initOta();  
    }
#endif

#ifdef ENABLE_DEBUG_I2C
    if (isSyslog) syslog(LOG_INFO, "--- i2cdetect\n");
    i2cdetect();  // default interval from 0x03 to 0x77
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif

#ifdef ENABLE_DEVICE_SERVO_MOTOR
    if (isSyslog) syslog(LOG_INFO, "--- init Servo motors\n");
    update_oled(NULL, "Init Servo motors", NULL, NULL);
    initPwm();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif // ENABLE_DEVICE_SERVO_MOTOR

#ifdef ENABLE_DEVICE_MOTOR
    if (isSyslog) syslog(LOG_INFO, "--- init motors\n");
    update_oled(NULL, "Init motors", NULL, NULL);
    motor1_controller.begin();
    motor2_controller.begin();
    motor3_controller.begin();
    motor4_controller.begin();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif // ENABLE_DEVICE_MOTOR

#ifdef ENABLE_DEVICE_IMU
    if (isSyslog) syslog(LOG_INFO, "--- init IMU\n");
    update_oled(NULL, "Init IMU", NULL, NULL);
    bool imu_ok = imu.init();
    if (!imu_ok) // take IMU failure as fatal
    {
        Serial.println("   IMU init failed");
        if (isSyslog) syslog(LOG_DEBUG, "   IMU init failed\n");
        update_oled(NULL, "Init IMU: KO", NULL, NULL);
        while (1)
        {
            flashLED(3); // flash 3 times
#if defined(ENABLE_DEVICE_WIFI) && defined(ENABLE_OTA)
            if (wifiDevice.isReady()) { runOta();  }
#endif
        }
    }
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif

#ifdef ENABLE_DEVICE_MAG
    if (isSyslog) syslog(LOG_INFO, "--- init Mag\n");
    update_oled(NULL, "Init MAG", NULL, NULL);
    bool mag_ok = mag.init();
    if (!mag_ok) // take IMU failure as fatal
    {
        Serial.println("MAG init failed");
        if (isSyslog) syslog(LOG_DEBUG, "%s MAG init failed %lu", String(__FUNCTION__), millis());
        while (1)
        {
            flashLED(4); // flash 4 times
#if defined(ENABLE_DEVICE_WIFI) && defined(ENABLE_OTA)
            if (wifiDevice.isReady()) { runOta();  }
#endif
        }
    }
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif // ENABLE_DEVICE_MAG

#ifdef ENABLE_DEVICE_BATTERY
    if (isSyslog) syslog(LOG_INFO, "--- init Battery\n");
    update_oled(NULL, "Init Battery", NULL, NULL);
    battery.initBattery();
    battery.readBattery();
    prev_voltage = battery.getVoltage();
    prev_current = battery.getCurrent();
    if (isSyslog) syslog(LOG_DEBUG, ("   battery: " + String(prev_voltage) + "V, "+ String(prev_current) + "A\n").c_str());
    update_oled(NULL, "Ready", ("batt: " + String(prev_voltage)+"V, "+ String(prev_current)+"A").c_str(),NULL);
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif

#ifdef ENABLE_DEVICE_ULTRASONIC
    if (isSyslog) syslog(LOG_INFO, "--- init ultrasonic sensor\n");
    update_oled(NULL, "Init ultrasonic", NULL, NULL);
    range.initRange();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif // ENABLE_DEVICE_ULTRASONIC

#ifdef ENABLE_DEVICE_ENCODER
    if (isSyslog) syslog(LOG_INFO, "--- init encoders\n");
    update_oled(NULL, "Init encoders", NULL, NULL);
    jointState[0].name = "joint_motor1";
    jointState[1].name = "joint_motor2";
    jointState[2].name = "joint_motor3";
    jointState[3].name = "joint_motor4";
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif // ENABLE_DEVICE_ENCODER

#ifdef ENABLE_DEVICE_LIDAR
    if (isSyslog) syslog(LOG_INFO, "--- init lidar\n");
    update_oled(NULL, "Init lidar", NULL, NULL);
    initLidar();
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif //ENABLE_DEVICE_LIDAR

#ifdef ENABLE_CONNECTOR_WEB
    if (isSyslog) syslog(LOG_INFO, "--- init Agent Web\n");
    update_oled(NULL, "Init Web Agent", NULL, NULL);
    connectorWeb.setKinematics(&kinematics);
    connectorWeb.setPID(K_P, K_I, K_D);
    connectorWeb.initAgent(NULL,*twistCallback,NULL,*pidCallback);
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n"); 
#endif // ENABLE_CONNECTOR_WEB

#ifdef ENABLE_CONNECTOR_ROS
    if (isSyslog) syslog(LOG_INFO, "--- init Agent ROS\n");
    update_oled(NULL, "Init ROS Agent", NULL, NULL);
    connectorROS.initAgent(*controlCallback,NULL,*jointCallback,NULL);
    connectorROS.setPID(K_P, K_I, K_D);
    connectorROS.syncTime(); // synchronize time with the agent
    if (isSyslog) syslog(LOG_DEBUG, "... Done\n");
#endif // ENABLE_CONNECTOR_ROS


#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif
    update_oled(NULL, "Ready", NULL, NULL);

    if (isSyslog) syslog(LOG_INFO, "=========END setup() =========\n");
}

void loop() {
#ifdef ENABLE_DEVICE_BATTERY
    battery.readBattery();
    prev_voltage = battery.getVoltage() * 0.01 + prev_voltage * 0.99;
    prev_current = battery.getCurrent() * 0.01 + prev_current * 0.99;
#endif

#ifdef ENABLE_CONNECTOR_WEB
    connectorWeb.listenAgent(0);
#endif // ENABLE_CONNECTOR_WEB

#ifdef ENABLE_CONNECTOR_ROS
    if (connectorROS.pingAgent(100,1))
       connectorROS.listenAgent(0);
    else
       fullStop();
#endif // ENABLE_CONNECTOR_ROS

#if defined(ENABLE_DEVICE_WIFI) && defined(ENABLE_OTA)
    if (wifiDevice.isReady()) { runOta();  }
#endif

#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
#ifdef BOARD_LOOP // board specific loop
    BOARD_LOOP
#endif

#ifdef ENABLE_DEVICE_BATTERY
    prev_voltage = battery.getVoltage() * 0.01 + prev_voltage * 0.99;
    prev_current = battery.getCurrent() * 0.01 + prev_current * 0.99;
#endif
}
