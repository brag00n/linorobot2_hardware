// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
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

#ifndef MAG_INTERFACE
#define MAG_INTERFACE

#include "config.h"

#ifndef MAG_COV
#define MAG_COV { 0.00001, 0.00001, 0.00001 }
#endif

class MAGInterface
{
    public:
        struct Vector_t {
            float x;
            float y;
            float z;
        };
        struct Mag_t{
            MAGInterface::Vector_t magnetic_field;
            float magnetic_field_covariance[9];
            #ifdef ENABLE_MICRO_ROS
            std_msgs__msg__Header header;
            #endif
        };
    protected:
    Mag_t mag_msg_;
        const float mag_cov[3] = MAG_COV;

    public:
        MAGInterface()
        {
            #ifdef ENABLE_MICRO_ROS
            mag_msg_.header.frame_id = micro_ros_string_utilities_set(mag_msg_.header.frame_id, "imu_link");
            #endif
        }

        virtual MAGInterface::Vector_t readMagnetometer() = 0;
        virtual bool startSensor() = 0;

        bool init()
        {
            bool sensor_ok = startSensor();
            return sensor_ok;
        }

        MAGInterface& readMAG()
        {
            mag_msg_.magnetic_field = readMagnetometer();
            mag_msg_.magnetic_field_covariance[0] = mag_cov[0];
            mag_msg_.magnetic_field_covariance[4] = mag_cov[1];
            mag_msg_.magnetic_field_covariance[8] = mag_cov[2];

#ifdef MAG_TWEAK
            MAG_TWEAK
#endif
            return *this;
        }
        Mag_t getData()
        {
            return mag_msg_;
        }
};

#endif
