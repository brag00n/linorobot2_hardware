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

#ifndef LINO_BASE_CONFIG_H
#define LINO_BASE_CONFIG_H

#define LED_PIN 13 //used for debugging status

//uncomment the base you're building
#define LINO_BASE DIFFERENTIAL_DRIVE       // 2WD and Tracked robot w/ 2 motors
// #define LINO_BASE SKID_STEER            // 4WD robot
// #define LINO_BASE MECANUM               // Mecanum drive robot

//uncomment the motor driver you're using
//#define USE_GENERIC_2_IN_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019
// #define USE_GENERIC_1_IN_MOTOR_DRIVER   // Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.
// #define USE_BTS7960_MOTOR_DRIVER        // BTS7970 Motor Driver
// #define USE_ESC_MOTOR_DRIVER            // Motor ESC for brushless motors
#define USE_RASPIMOTORHAT_MOTOR_DRIVER      // Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin ie. L298, L293, VNH5019

//uncomment the IMU you're using
//#define USE_GY85_IMU
#define USE_MPU6050_IMU
// #define USE_MPU9150_IMU
// #define USE_MPU9250_IMU

//Encoder
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
/*
#define K_P 0.6                             // P constant
#define K_I 0.8                             // I constant
#define K_D 0.5                             // D constant
i*/
//#define K_P 0.01                             // P constant
#define K_P 0.2                            // P constant
#define K_I 0.1                            // I constant
#define K_D 0.1                             // D constant

#define NED_TO_ENU false

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)  
         BACK
*/

//ROS:
#define ROS_DOMAIN_ID 0
#define NODE "base_node"
#define NAMESPACE ""
#define BASE_FRAME_ID "base_footprint"
#define ODOM_FRAME_ID "odom"
#define IMU_FRAME_ID  "imu_link"
#define MOTOR1 "joint_lf"
#define MOTOR2 "joint_rf"
#define MOTOR3 "joint_lb"
#define MOTOR4 "joint_rb"
#define UPDATE_FREQ 50  // Control timer frequency in Hz (50Hz = 20ms)

//define your robot' specs here
#define MOTOR_MAX_RPM 1440                  // motor's max RPM          
#define MAX_RPM_RATIO 0.85                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 6          // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24          // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power connected to the motor (used for calibration)
#define COUNTS_PER_REV1 48 //12             wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 48 //12              // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 48 //12              // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 48 //12              // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.065                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.135            // distance between left and right wheels
#define PWM_BITS 8                          // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 580000           // PWM Frequency in Hz

// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV true 
#define MOTOR2_ENCODER_INV false
#define MOTOR3_ENCODER_INV true 
#define MOTOR4_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV false
#define MOTOR4_INV false

// ENCODER PINS
#define MOTOR1_ENCODER_A 3
#define MOTOR1_ENCODER_B 2 

#define MOTOR2_ENCODER_A 21
#define MOTOR2_ENCODER_B 20

#define MOTOR3_ENCODER_A 23
#define MOTOR3_ENCODER_B 22

#define MOTOR4_ENCODER_A 5
#define MOTOR4_ENCODER_B 4

// MOTOR PINS
#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can swap it with pin no 1 instead.
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B 1

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B 8

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 3
  #define MOTOR4_IN_B 2

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_GENERIC_1_IN_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_A 20
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A 6
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR3_PWM 22
  #define MOTOR3_IN_A 23
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A 3
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif 

#ifdef USE_BTS7960_MOTOR_DRIVER
  #define MOTOR1_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_A 21 // Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 instead.
  #define MOTOR1_IN_B 20 // Pin no 20 is not a PWM pin on Teensy 4.x, you can use pin no 0 instead.

  #define MOTOR2_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_A 5
  #define MOTOR2_IN_B 6

  #define MOTOR3_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_A 22
  #define MOTOR3_IN_B 23

  #define MOTOR4_PWM -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_A 4
  #define MOTOR4_IN_B 3

  #define PWM_MAX pow(2, PWM_BITS) - 1
  #define PWM_MIN -PWM_MAX
#endif

#ifdef USE_ESC_MOTOR_DRIVER
  #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x. You can use pin no 1 instead.
  #define MOTOR1_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR2_PWM 5
  #define MOTOR2_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR3_PWM 22 
  #define MOTOR3_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define MOTOR4_PWM 4
  #define MOTOR4_IN_A -1 //DON'T TOUCH THIS! This is just a placeholder
  #define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

  #define PWM_MAX 400
  #define PWM_MIN -PWM_MAX
#endif

// MOTOR PINS
#ifdef USE_RASPIMOTORHAT_MOTOR_DRIVER
  #define MOTOR1_PWM 0 //Pin no 21 is not a PWM pin on Teensy 4.x, you can swap it with pin no 1 instead.
  #define MOTOR1_IN_A 0
  #define MOTOR1_IN_B 0

  #define MOTOR2_PWM 1
  #define MOTOR2_IN_A 0
  #define MOTOR2_IN_B 0

  #define MOTOR3_PWM 2
  #define MOTOR3_IN_A 0
  #define MOTOR3_IN_B 0

  #define MOTOR4_PWM 3
  #define MOTOR4_IN_A 0
  #define MOTOR4_IN_B 0

  #define PWM_MAX 255
  #define PWM_MIN -255
#endif 

#endif
