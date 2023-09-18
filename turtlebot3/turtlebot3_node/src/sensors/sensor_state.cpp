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

#include "turtlebot3_node/sensors/sensor_state.hpp"

#include <memory>
#include <string>
#include <utility>

using robotis::turtlebot3::sensors::SensorState;

SensorState::SensorState(
  std::shared_ptr<rclcpp::Node> & nh,
  const std::string & topic_name,
  const uint8_t & bumper_forward,
  const uint8_t & bumper_backward,
  const uint8_t & illumination,
  const uint8_t & cliff,
  const uint8_t & sonar)
: Sensors(nh),
  bumper_forward_(bumper_forward),
  bumper_backward_(bumper_backward),
  illumination_(illumination),
  cliff_(cliff),
  sonar_(sonar)
{
  pub_ = nh->create_publisher<turtlebot3_msgs::msg::SensorState>(topic_name, this->qos_);

  RCLCPP_INFO(nh_->get_logger(), "Succeeded to create sensor state publisher");
}

void SensorState::publish(
  const rclcpp::Time & now,
  std::shared_ptr<DynamixelSDKWrapper> & dxl_sdk_wrapper)
{
  auto msg = std::make_unique<turtlebot3_msgs::msg::SensorState>();

  msg->header.stamp = now;
  msg->bumper = 0;
  msg->cliff = 0.0f;
  msg->sonar = 0.0f;
  msg->illumination = 0.0f;

  // update button state
  uint8_t button_push_state;
  uint8_t button_0_state;
  uint8_t button_1_state;

  msg->button = 0;

  // update torque enable state
  msg->torque = true;

  msg->left_encoder = 0.0f;

  msg->right_encoder = 0.0f;

  msg->battery = 0.01f * 1000;

  pub_->publish(std::move(msg));
}
