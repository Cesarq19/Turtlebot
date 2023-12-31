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

#ifndef TURTLEBOT3_NODE__SENSORS__SENSORS_HPP_
#define TURTLEBOT3_NODE__SENSORS__SENSORS_HPP_

#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

// #include "node_turtlebot/dynamixel_sdk_wrapper.hpp"
#include "node_turtlebot/core_motores.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

class Sensors
{
public:
  explicit Sensors(
    std::shared_ptr<rclcpp::Node> & nh,
    const std::string & frame_id = "")
  : nh_(nh),
    frame_id_(frame_id)
  {
  }

  virtual void publish(
    const rclcpp::Time & now) = 0;

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::string frame_id_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));
};

#endif  // TURTLEBOT3_NODE__SENSORS__SENSORS_HPP_