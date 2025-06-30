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

#include "odometry.h"

Odometry::Odometry():
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0)
{
    // odometry_data_.header.frame_id = micro_ros_string_utilities_set(odometry_data_.header.frame_id, "odom");
    // odometry_data_.child_frame_id = micro_ros_string_utilities_set(odometry_data_.child_frame_id, "base_footprint");
}

void Odometry::update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z)
{
    float delta_heading = angular_vel_z * vel_dt; //radians
    float cos_h = cos(heading_);
    float sin_h = sin(heading_);
    float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; //m
    float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; //m
    const float pose_cov[6] = POSE_COV;
    const float twist_cov[6] = TWIST_COV;

    //calculate current position of the robot
    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    //calculate robot's heading in quaternion angle
    //ROS has a function to calculate yaw in quaternion angle
    float q[4];
    euler_to_quat(0, 0, heading_, q);

    //robot's position in x,y, and z
    odometry_data_.pose_pose_position_x = x_pos_;
    odometry_data_.pose_pose_position_y = y_pos_;
    odometry_data_.pose_pose_position_z = 0.0;

    //robot's heading in quaternion
    odometry_data_.pose_pose_orientation_x = (double) q[1];
    odometry_data_.pose_pose_orientation_y = (double) q[2];
    odometry_data_.pose_pose_orientation_z = (double) q[3];
    odometry_data_.pose_pose_orientation_w = (double) q[0];

    odometry_data_.pose_covariance[0] = pose_cov[0];
    odometry_data_.pose_covariance[7] = pose_cov[1];
    odometry_data_.pose_covariance[14] = pose_cov[2];
    odometry_data_.pose_covariance[21] = pose_cov[3];
    odometry_data_.pose_covariance[28] = pose_cov[4];
    odometry_data_.pose_covariance[35] = pose_cov[5];

    //linear speed from encoders
    odometry_data_.twist_twist_linear_x = linear_vel_x;
    odometry_data_.twist_twist_linear_y = linear_vel_y;
    odometry_data_.twist_twist_linear_z = 0.0;

    //angular speed from encoders
    odometry_data_.twist_twist_angular_x = 0.0;
    odometry_data_.twist_twist_angular_y = 0.0;
    odometry_data_.twist_twist_angular_z = angular_vel_z;

    odometry_data_.twist_covariance[0] = twist_cov[0];
    odometry_data_.twist_covariance[7] = twist_cov[1];
    odometry_data_.twist_covariance[14] = twist_cov[2];
    odometry_data_.twist_covariance[21] = twist_cov[3];
    odometry_data_.twist_covariance[28] = twist_cov[4];
    odometry_data_.twist_covariance[35] = twist_cov[5];
}

Odometry::Odometry_data Odometry::getData()
{
    return odometry_data_;
}

const void Odometry::euler_to_quat(float roll, float pitch, float yaw, float* q) 
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}
