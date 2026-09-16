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

#ifndef IMU_INTERFACE
#define IMU_INTERFACE

#include "config.h"
#include "Device.hpp"

#ifndef ACCEL_COV
#define ACCEL_COV { 0.00001, 0.00001, 0.00001 }
#endif
#ifndef GYRO_COV
#define GYRO_COV { 0.00001, 0.00001, 0.00001 }
#endif
#ifndef ORI_COV
#define ORI_COV { 0.00001, 0.00001, 0.00001 }
#endif

class IMUInterface{
    public:
        struct Vector_t {
            float x;
            float y;
            float z;
            float w;
        };
        struct Imu_t: Device::Device_t {
            IMUInterface::Vector_t angular_velocity;
            IMUInterface::Vector_t linear_acceleration;
            IMUInterface::Vector_t orientation;
            float magnetic_field[3];
            float angular_velocity_covariance[9];
            float linear_acceleration_covariance[9];
            float orientation_covariance[9];
        };
    protected:
        Imu_t imu_msg_;
        const float g_to_accel_ = 9.81;
        const float mgauss_to_utesla_ = 0.1;
        const float utesla_to_tesla_ = 0.000001;

        const float accel_cov[3] = ACCEL_COV;
        const float gyro_cov[3] = GYRO_COV;
        const float ori_cov[3] = ORI_COV;
        const int sample_size_ = 40;

        IMUInterface::Vector_t gyro_cal_;

        void calibrateGyro()
        {
            IMUInterface::Vector_t gyro;

            // Remise a zero de l'accumulateur AVANT la moyenne : garantit un biais
            // correct au 1er appel (membre non initialise) et surtout lors des
            // recalibrages a la demande (sinon les biais successifs s'additionnent).
            gyro_cal_.x = 0.0f;
            gyro_cal_.y = 0.0f;
            gyro_cal_.z = 0.0f;

            for(int i=0; i<sample_size_; i++)
            {
                gyro = readGyroscope();
                gyro_cal_.x += gyro.x;
                gyro_cal_.y += gyro.y;
                gyro_cal_.z += gyro.z;

                delay(50);
            }

            gyro_cal_.x = gyro_cal_.x / (float)sample_size_;
            gyro_cal_.y = gyro_cal_.y / (float)sample_size_;
            gyro_cal_.z = gyro_cal_.z / (float)sample_size_;
        }

        IMUInterface()
        {
            // #ifdef ENABLE_MICRO_ROS
            // imu_msg_.header.frame_id = micro_ros_string_utilities_set(imu_msg_.header.frame_id, "imu_link");
            // #endif
        }

        virtual IMUInterface::Vector_t readAccelerometer() = 0;
        virtual IMUInterface::Vector_t readGyroscope() = 0;
        virtual bool startSensor() = 0;

    public:
        bool init()
        {
            bool sensor_ok = startSensor();
            if(sensor_ok)
                calibrateGyro();

            return sensor_ok;
        }

        Imu_t getData()
        {
            return imu_msg_;
        }

        // Recalibrage du biais gyro A LA DEMANDE (robot immobile) : re-mesure la moyenne
        // du gyroscope et la retranche des lectures suivantes. Exposee pour le handler
        // MAVLink MAV_CMD_PREFLIGHT_CALIBRATION (re-zerotage du cap sans reboot). Bloque
        // ~2 s (sample_size_ x delay(50)) ; a n'appeler qu'a l'arret.
        void recalibrateGyro()
        {
            calibrateGyro();
        }

        IMUInterface& readIMU()
        {
            imu_msg_.angular_velocity = readGyroscope();
#ifndef USE_MPU6050_IMU // mpu6050 already calibrated in driver
            imu_msg_.angular_velocity.x -= gyro_cal_.x;
            imu_msg_.angular_velocity.y -= gyro_cal_.y;
            imu_msg_.angular_velocity.z -= gyro_cal_.z;
#endif

            if(imu_msg_.angular_velocity.x > -0.01 && imu_msg_.angular_velocity.x < 0.01 )
                imu_msg_.angular_velocity.x = 0;

            if(imu_msg_.angular_velocity.y > -0.01 && imu_msg_.angular_velocity.y < 0.01 )
                imu_msg_.angular_velocity.y = 0;

            if(imu_msg_.angular_velocity.z > -0.01 && imu_msg_.angular_velocity.z < 0.01 )
                imu_msg_.angular_velocity.z = 0;

            imu_msg_.angular_velocity_covariance[0] = gyro_cov[0];
            imu_msg_.angular_velocity_covariance[4] = gyro_cov[1];
            imu_msg_.angular_velocity_covariance[8] = gyro_cov[2];

            imu_msg_.linear_acceleration = readAccelerometer();
            imu_msg_.linear_acceleration_covariance[0] = accel_cov[0];
            imu_msg_.linear_acceleration_covariance[4] = accel_cov[1];
            imu_msg_.linear_acceleration_covariance[8] = accel_cov[2];

            imu_msg_.orientation_covariance[0] = ori_cov[0];
            imu_msg_.orientation_covariance[4] = ori_cov[1];
            imu_msg_.orientation_covariance[8] = ori_cov[2];

#if defined(ENABLE_CONNECTOR_SERIAL_FRAME) || defined(ENABLE_MAVLINK)
            // --- Estimation d'orientation (filtre complementaire) ---
            // Le wrapper QMI8658IMU (robot #2 WaveShare) ne calcule aucune orientation :
            // sans ce bloc, imu_msg_.orientation reste (0,0,0,0) et publishImu (trames binaires
            // 0x0C OU ATTITUDE #30 MAVLink) emet roll/pitch/yaw = 0 (bug HUD « IMU +0.0 »).
            // On l'estime donc ici, a partir des grandeurs deja remplies ci-dessus :
            //   - accelerometre (m/s^2) -> roll/pitch ABSOLUS via la gravite (basse freq) ;
            //   - gyroscope (rad/s, deja calibre) -> integration (haute freq) + cap yaw
            //     RELATIF (pas de reference absolue tant que le magneto n'est pas fusionne).
            // Melange complementaire (alpha) puis conversion en quaternion (ordre ZYX) que
            // publishImu reconvertit en angles d'Euler. Garde sous les DEUX transports app
            // (ConnectorSerialFrame et ConnectorMavlink, qui n'ont pas de fusion propre) mais
            // PAS sous micro-ROS (nav ROS avec sa propre fusion en aval).
            {
                static uint32_t last_us = 0;
                static float est_roll = 0.0f, est_pitch = 0.0f, est_yaw = 0.0f;
                const uint32_t now_us = micros();
                // (now_us - last_us) gere naturellement le wrap u32 (arithmetique modulaire).
                float dt = (last_us == 0) ? 0.0f : (float)(now_us - last_us) * 1e-6f;
                last_us = now_us;
                if (dt < 0.0f || dt > 0.5f) dt = 0.0f; // garde-fou (1er appel, pause, overflow)

                const float ax = imu_msg_.linear_acceleration.x;
                const float ay = imu_msg_.linear_acceleration.y;
                const float az = imu_msg_.linear_acceleration.z;
                const float gx = imu_msg_.angular_velocity.x; // rad/s
                const float gy = imu_msg_.angular_velocity.y;
                const float gz = imu_msg_.angular_velocity.z;

                // roll/pitch absolus depuis la gravite (rad)
                const float acc_roll  = atan2f(ay, az);
                const float acc_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

                // integration gyro (haute frequence)
                est_roll  += gx * dt;
                est_pitch += gy * dt;
                est_yaw   += gz * dt;

                // melange complementaire sur roll/pitch ; l'accel n'est fiable que proche de 1 g
                // (on la rejette en chute libre / choc, |a| trop faible ou trop fort).
                const float amag2 = ax * ax + ay * ay + az * az;
                const bool  acc_ok = (amag2 > 16.0f && amag2 < 400.0f); // ~ [4, 20] m/s^2
                const float alpha = 0.98f;
                if (dt == 0.0f) {                 // amorcage : cale sur l'accel
                    est_roll = acc_roll; est_pitch = acc_pitch;
                } else if (acc_ok) {
                    est_roll  = alpha * est_roll  + (1.0f - alpha) * acc_roll;
                    est_pitch = alpha * est_pitch + (1.0f - alpha) * acc_pitch;
                }

                // normalise le yaw dans [-PI, PI]
                if (est_yaw >  PI) est_yaw -= 2.0f * PI;
                else if (est_yaw < -PI) est_yaw += 2.0f * PI;

                // Euler (roll,pitch,yaw) -> quaternion, ordre ZYX
                const float cr = cosf(est_roll * 0.5f),  sr = sinf(est_roll * 0.5f);
                const float cp = cosf(est_pitch * 0.5f), sp = sinf(est_pitch * 0.5f);
                const float cy = cosf(est_yaw * 0.5f),   sy = sinf(est_yaw * 0.5f);
                imu_msg_.orientation.w = cr * cp * cy + sr * sp * sy;
                imu_msg_.orientation.x = sr * cp * cy - cr * sp * sy;
                imu_msg_.orientation.y = cr * sp * cy + sr * cp * sy;
                imu_msg_.orientation.z = cr * cp * sy - sr * sp * cy;
            }
#endif // ENABLE_CONNECTOR_SERIAL_FRAME || ENABLE_MAVLINK

#ifdef IMU_TWEAK
            IMU_TWEAK
#endif
            return *this;
        }
};

#endif
