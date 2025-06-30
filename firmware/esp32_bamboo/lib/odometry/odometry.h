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

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "config.h"
#include "Device.hpp"

#ifndef POSE_COV
#define POSE_COV { 0.0001, 0.0001, 0, 0, 0, 0.0001 }
#endif
#ifndef TWIST_COV
#define TWIST_COV { 0.00001, 0.00001, 0, 0, 0, 0.00001 }
#endif

class Odometry
{
    public:
        struct Odometry_data: Device::Device_t
            {   
                float pose_pose_position_x;
                float pose_pose_position_y;
                float pose_pose_position_z;
                float pose_pose_orientation_x;
                float pose_pose_orientation_y;
                float pose_pose_orientation_z;
                float pose_pose_orientation_w;
                float pose_covariance[36];
                float twist_twist_linear_x;
                float twist_twist_linear_y;
                float twist_twist_linear_z;
                float twist_twist_angular_x;
                float twist_twist_angular_y;
                float twist_twist_angular_z;
                float twist_covariance[36];

            };
        Odometry();
        void update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z);
        Odometry_data getData();

    private:
        const void euler_to_quat(float x, float y, float z, float* q);

        Odometry_data odometry_data_;
        float x_pos_;
        float y_pos_;
        float heading_;
};

#endif
