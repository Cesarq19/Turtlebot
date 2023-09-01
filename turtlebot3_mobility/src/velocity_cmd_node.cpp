#include <cstdio>
#include <memory>
#include <string>

#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>

#include "velocity_cmd_node.hpp"

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600             // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_velocity_right = 0;
uint32_t goal_velocity_left = 0;
int dxl_comm_result = COMM_TX_FAIL;

VelocityCmdNode::VelocityCmdNode()
    : Node("velocity_cmd_node")
{
    RCLCPP_INFO(this->get_logger(), "Run motors node");
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);

    const auto qos =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        qos,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
        {
            uint8_t dxl_error = 0;

            uint32_t linear_velocity = (unsigned int)msg->linear.x * 100;
            uint32_t angular_velocity = (unsigned int)msg->angular.z * 100;

            goal_velocity_left = linear_velocity + angular_velocity;
            goal_velocity_right = linear_velocity - angular_velocity;

            dxl_comm_result =
                packetHandler->write4ByteTxRx(
                    portHandler,
                    0,
                    ADDR_GOAL_VELOCITY,
                    goal_velocity_right,
                    &dxl_error);

            dxl_comm_result =
                packetHandler->write4ByteTxRx(
                    portHandler,
                    1,
                    ADDR_GOAL_VELOCITY,
                    goal_velocity_left,
                    &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
                RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Todo OK");
            }
        });
}

VelocityCmdNode::~VelocityCmdNode()
{
}

void setupDynamixel()
{
    // Use Position Control Mode
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        0,
        ADDR_OPERATING_MODE,
        3,
        &dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        1,
        ADDR_OPERATING_MODE,
        3,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to set Position Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to set Position Control Mode.");
    }

    // Enable Torque of DYNAMIXEL
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        0,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        0,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);

    if (dxl_comm_result != COMM_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("velocity_cmd_node"), "Failed to enable torque.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("velocity_cmd_node"), "Succeeded to enable torque.");
    }
}

int main(int argc, char *argv[])
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

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

    setupDynamixel();

    rclcpp::init(argc, argv);

    auto velocityCmdNode = std::make_shared<VelocityCmdNode>();
    rclcpp::spin(velocityCmdNode);
    rclcpp::shutdown();

    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);

    return 0;
}
