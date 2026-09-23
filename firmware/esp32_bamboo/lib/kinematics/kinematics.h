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

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "Arduino.h"

#define RPM_TO_RPS 1/60

class Kinematics
{
    public:
        enum base {DIFFERENTIAL_DRIVE, SKID_STEER, MECANUM};

        base base_platform_;

        struct rpm
        {
            float motor1;
            float motor2;
            float motor3;
            float motor4;
        };
        
        struct velocities
        {
            float linear_x;
            float linear_y;
            float angular_z;
        };

        struct pwm
        {
            int motor1;
            int motor2;
            int motor3;
            int motor4;
        };
        Kinematics(base robot_base, int motor_max_rpm, float max_rpm_ratio,
                   float motor_operating_voltage, float motor_power_max_voltage,
                   float wheel_diameter, float wheels_y_distance);
        velocities getVelocities(float rpm1, float rpm2, float rpm3, float rpm4);
        rpm getRPM(float linear_x, float linear_y, float angular_z);
        float getMaxRPM();

        // --- geometrie inscriptible a chaud (SRAM) ---------------------------
        // La cinematique n'est plus figee a la compilation : l'hote peut la
        // corriger par MAVLink (PARAM_SET WHEEL_CIRC / WHEEL_APB / CAR_TYPE)
        // sans reflasher. Aucune reconstruction d'objet n'est necessaire, les
        // membres sont relus a chaque appel (calculateRPM / getVelocities).
        // Les valeurs non strictement positives sont ignorees : un parametre
        // absurde venu du fil ne doit pas immobiliser le robot.
        void setWheelDiameter(float wheel_diameter);
        void setWheelsYDistance(float wheels_y_distance);
        void setMotorMaxRPM(int motor_max_rpm);
        void setBase(base robot_base);
        float getWheelDiameter();
        float getWheelsYDistance();
        base getBase();

    private:
        rpm calculateRPM(float linear_x, float linear_y, float angular_z);
        int getTotalWheels(base robot_base);
        void updateMaxRPM();

        float max_rpm_;
        float wheels_y_distance_;
        float pwm_res_;
        float wheel_circumference_;
        int total_wheels_;
        // memorises pour pouvoir recalculer max_rpm_ apres un setMotorMaxRPM
        int motor_max_rpm_;
        float max_rpm_ratio_;
        float motor_operating_voltage_;
        float motor_power_max_voltage_;
};

#endif