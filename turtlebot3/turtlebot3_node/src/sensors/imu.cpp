// Copyright 2019 ROBOTIS CO., LTD.
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
//
// Author: Darby Lim

#include "turtlebot3_node/sensors/imu.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::Imu;

Imu::Imu(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & imu_topic_name,
  const std::string & mag_topic_name,
  const std::string & frame_id)
: Sensors(nh, frame_id)
{
  imu_pub_ = nh->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, this->qos_);
  mag_pub_ = nh->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create imu publisher");
}

void Imu::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

  imu_msg->header.frame_id = this->frame_id_;
  imu_msg->header.stamp = now;

  auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();

  mag_msg->header.frame_id = this->frame_id_;
  mag_msg->header.stamp = now;

  imu_pub_->publish(std::move(imu_msg));
  mag_pub_->publish(std::move(mag_msg));
}
