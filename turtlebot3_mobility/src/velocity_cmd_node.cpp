#include <cstdio>
#include <memory>
#include <string>

#include "turtlebot3_mobility/velocity_cmd_node.hpp"
#include "turtlebot3_mobility/diff_drive_controller.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

// Control table address
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define MOTOR_LEFT_ID 1
#define MOTOR_RIGHT_ID 0
#define BAUDRATE 57600                                                                                // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

using ramel::tb3Custom::VelocityCmdNode;
using namespace dynamixel;

PortHandler *portHandler;
PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint32_t goal_velocity = 0;

VelocityCmdNode::VelocityCmdNode() : Node("velocity_cmd_node")
{
    RCLCPP_INFO(this->get_logger(), "Run mobility node");

    add_sensors();
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
        {
            uint8_t dxl_error = 0;

            linear_velocity_ = static_cast<int32_t>(msg->linear.x * 100);
            angular_velocity_ = static_cast<int32_t>(msg->angular.z * 100);

            int32_t velocity_left = linear_velocity_ - angular_velocity_;
            int32_t velocity_right = linear_velocity_ + angular_velocity_;

            // Write Goal Velocity
            dxl_comm_result =
                packetHandler->write4ByteTxRx(
                    portHandler,
                    MOTOR_LEFT_ID,
                    ADDR_GOAL_VELOCITY,
                    velocity_left,
                    &dxl_error);

            dxl_comm_result =
                packetHandler->write4ByteTxRx(
                    portHandler,
                    MOTOR_RIGHT_ID,
                    ADDR_GOAL_VELOCITY,
                    -velocity_right,
                    &dxl_error);

            RCLCPP_INFO(
                this->get_logger(),
                "left_vel: %d right_vel: %d", velocity_left, velocity_right);
        });
}

VelocityCmdNode::~VelocityCmdNode()
{
}

void VelocityCmdNode::add_sensors()
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
    new sensors::BatteryState(
      node_handle_,
      "battery_state"));

  sensors_.push_back(
    new sensors::Imu(
      node_handle_,
      "imu",
      "magnetic_field",
      "imu_link"));

  sensors_.push_back(
    new sensors::SensorState(
      node_handle_,
      "sensor_state",
      is_connected_bumper_1,
      is_connected_bumper_2,
      is_connected_illumination,
      is_connected_ir,
      is_connected_sonar));

  sensors_.push_back(new sensors::JointState(node_handle_, "joint_states", "base_link"));
}

void setupDynamixel(uint8_t dxl_id1, uint8_t dxl_id2)
{
    // Use Velocity Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id1,
        ADDR_OPERATING_MODE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to set Velocity Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to set Velocity Control Mode.");
    }

    // Use Velocity Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id2,
        ADDR_OPERATING_MODE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to set Velocity Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to set Velocity Control Mode.");
    }
}

int main(int argc, char *argv[])
{
    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to open the port!");
        return -1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to set the baudrate!");
        return -1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to set the baudrate.");
    }
    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        MOTOR_LEFT_ID,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    packetHandler->write1ByteTxRx(
        portHandler,
        MOTOR_RIGHT_ID,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    // Use Velocity Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        MOTOR_LEFT_ID,
        ADDR_OPERATING_MODE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to set Velocity Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to set Velocity Control Mode.");
    }

    // Use Velocity Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        MOTOR_RIGHT_ID,
        ADDR_OPERATING_MODE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to set Velocity Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to set Velocity Control Mode.");
    }

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    auto velocitycmd = std::make_shared<ramel::tb3Custom::VelocityCmdNode>();

    auto diff_drive_controller = std::make_shared<ramel::tb3Custom::DiffDriveController>(
        0.160,
        0.033);

    executor.add_node(velocitycmd);
    executor.add_node(diff_drive_controller);
    
    //rclcpp::spin(velocitycmd);
    executor.spin();
    rclcpp::shutdown();

    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        MOTOR_LEFT_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    packetHandler->write1ByteTxRx(
        portHandler,
        MOTOR_RIGHT_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    return 0;
}
