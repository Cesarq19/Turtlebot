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

#include "turtlebot3_node/turtlebot3.hpp"

#include <memory>
#include <string>

using robotis::turtlebot3::TurtleBot3;
using namespace std::chrono_literals;

TurtleBot3::TurtleBot3(const std::string &usb_port)
    : Node("turtlebot3_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  init_dynamixel_sdk_wrapper(usb_port);

  add_motors();
  add_wheels();
  add_sensors();

  run();
}

TurtleBot3::Wheels *TurtleBot3::get_wheels()
{
  return &wheels_;
}

TurtleBot3::Motors *TurtleBot3::get_motors()
{
  return &motors_;
}

void TurtleBot3::init_dynamixel_sdk_wrapper(const std::string &usb_port)
{
  DynamixelSDKWrapper::Device opencr = {usb_port, 200, 57600, 2.0f};

  this->declare_parameter<uint8_t>("opencr.id");
  this->declare_parameter<int>("opencr.baud_rate");
  this->declare_parameter<float>("opencr.protocol_version");

  this->get_parameter_or<uint8_t>("opencr.id", opencr.id, 200);
  this->get_parameter_or<int>("opencr.baud_rate", opencr.baud_rate, 57600);
  this->get_parameter_or<float>("opencr.protocol_version", opencr.protocol_version, 2.0f);

  RCLCPP_INFO(this->get_logger(), "Init DynamixelSDKWrapper");

  dxl_sdk_wrapper_ = std::make_shared<DynamixelSDKWrapper>(opencr);
}

void TurtleBot3::check_device_status()
{
  if (dxl_sdk_wrapper_->is_connected_to_device())
  {
    RCLCPP_INFO(this->get_logger(), "Start Calibration of Gyro");
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(this->get_logger(), "Calibration End");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed connection with Devices");
    rclcpp::shutdown();
    return;
  }

  const int8_t NOT_CONNECTED_MOTOR = -1;

  int8_t device_status = dxl_sdk_wrapper_->get_data_from_device<int8_t>(
      extern_control_table.hardware_error_status.addr,
      extern_control_table.device_status.length);

  switch (device_status)
  {
  case NOT_CONNECTED_MOTOR:
    RCLCPP_WARN(this->get_logger(), "Please double check your Dynamixels and Power");
    break;

  default:
    break;
  }
}

void TurtleBot3::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter<float>("motors.profile_acceleration_constant");
  this->declare_parameter<float>("motors.profile_acceleration");

  this->get_parameter_or<float>(
      "motors.profile_acceleration_constant",
      motors_.profile_acceleration_constant,
      214.577);

  this->get_parameter_or<float>(
      "motors.profile_acceleration",
      motors_.profile_acceleration,
      0.0);
}

void TurtleBot3::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter<float>("wheels.separation");
  this->declare_parameter<float>("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

void TurtleBot3::add_sensors()
{
  RCLCPP_INFO(this->get_logger(), "Add Sensors");

  uint8_t is_connected_bumper_1 = 0;
  uint8_t is_connected_bumper_2 = 0;
  uint8_t is_connected_illumination = 0;
  uint8_t is_connected_ir = 0;
  uint8_t is_connected_sonar = 0;

  this->declare_parameter<uint8_t>("sensors.bumper_1");
  this->declare_parameter<uint8_t>("sensors.bumper_2");
  this->declare_parameter<uint8_t>("sensors.illumination");
  this->declare_parameter<uint8_t>("sensors.ir");
  this->declare_parameter<uint8_t>("sensors.sonar");

  this->get_parameter_or<uint8_t>(
      "sensors.bumper_1",
      is_connected_bumper_1,
      0);
  this->get_parameter_or<uint8_t>(
      "sensors.bumper_2",
      is_connected_bumper_2,
      0);
  this->get_parameter_or<uint8_t>(
      "sensors.illumination",
      is_connected_illumination,
      0);
  this->get_parameter_or<uint8_t>(
      "sensors.ir",
      is_connected_ir,
      0);
  this->get_parameter_or<uint8_t>(
      "sensors.sonar",
      is_connected_sonar,
      0);

  sensors_.push_back(
      new sensors::Imu(
          node_handle_,
          "imu",
          "magnetic_field",
          "imu_link"));

  sensors_.push_back(new sensors::JointState(node_handle_, "joint_states", "base_link"));
}

void TurtleBot3::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  heartbeat_timer(std::chrono::milliseconds(100));

  parameter_event_callback();
  cmd_vel_callback();
}

void TurtleBot3::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
      timeout,
      [this]() -> void
      {
        rclcpp::Time now = this->now();

        dxl_sdk_wrapper_->read_data_set();

        for (const auto &sensor : sensors_)
        {
          sensor->publish(now, dxl_sdk_wrapper_);
        }
      });
}

void TurtleBot3::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
      timeout,
      [this]() -> void
      {
        static uint8_t count = 0;
        std::string msg;

        // dxl_sdk_wrapper_->set_data_to_device(
        // extern_control_table.heartbeat.addr,
        // extern_control_table.heartbeat.length,
        //&count,
        //&msg);
        count++;
      });
}

void TurtleBot3::parameter_event_callback()
{
  priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
  {
    for (const auto &changed_parameter : event->changed_parameters)
    {
      RCLCPP_DEBUG(
          this->get_logger(),
          "changed parameter name : %s",
          changed_parameter.name.c_str());

      if (changed_parameter.name == "motors.profile_acceleration")
      {
        std::string sdk_msg;

        motors_.profile_acceleration =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();

        motors_.profile_acceleration =
            motors_.profile_acceleration / motors_.profile_acceleration_constant;

        union Data
        {
          int32_t dword[2];
          uint8_t byte[4 * 2];
        } data;

        data.dword[0] = static_cast<int32_t>(motors_.profile_acceleration);
        data.dword[1] = static_cast<int32_t>(motors_.profile_acceleration);

        uint16_t address = extern_control_table.profile_acceleration.addr;
        uint16_t length = extern_control_table.profile_acceleration.length;

        // Aceleracion motor izquierdo
        dxl_sdk_wrapper_->write_motors(0, address, length, data.dword[0], &sdk_msg);
        
        // Aceleracion motor izquierdo
        dxl_sdk_wrapper_->write_motors(1, address, length, data.dword[1], &sdk_msg);

        RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : %f [rev/min2] sdk_msg : %s",
            motors_.profile_acceleration,
            sdk_msg.c_str());
      }
    }
  };

  parameter_event_sub_ = priv_parameters_client_->on_parameter_event(param_event_callback);
}

void TurtleBot3::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
      {
        std::string sdk_msg;

        union Data
        {
          int32_t dword[6];
          uint8_t byte[4 * 6];
        } data;

        int32_t linear_x = static_cast<int32_t>(msg->linear.x * 100);
        int32_t angular_z = static_cast<int32_t>(msg->angular.z * 100);

        data.dword[0] = linear_x + angular_z;
        data.dword[1] = 0;
        data.dword[2] = 0;
        data.dword[3] = 0;
        data.dword[4] = 0;
        data.dword[5] = -(linear_x - angular_z);

        uint16_t start_addr = extern_control_table.goal_velocity.addr;
        uint16_t addr_length = 4;

        // Velocidad motor izquierdo
        dxl_sdk_wrapper_->write_motors(0, start_addr, addr_length, 2 * data.dword[0], &sdk_msg);
        
        // Velocidad motor izquierdo
        dxl_sdk_wrapper_->write_motors(1, start_addr, addr_length, 2 * data.dword[5], &sdk_msg);

        RCLCPP_DEBUG(
            this->get_logger(),
            "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
      });
}
