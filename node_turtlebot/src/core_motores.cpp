#include "node_turtlebot/core_motores.hpp"

#include <memory>
#include <string>

using namespace dynamixel;
using namespace std::chrono_literals;

void Motores::initMotors()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motores_node"), "Failed to open the port!");
        //return -1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("motores_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);

    if (dxl_comm_result == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motores_node"), "Failed to set the baudrate!");
        //return -1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("motores_node"), "Succeeded to set the baudrate.");
    }

    // Enable torque left motor
    packetHandler->write1ByteTxRx(
        portHandler,
        MOTOR_LEFT_ID,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error);
    // Enable torque right motor
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
        RCLCPP_ERROR(rclcpp::get_logger("motores_node"), "Failed to set Velocity Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("motores_node"), "Succeeded to set Velocity Control Mode.");
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
        RCLCPP_ERROR(rclcpp::get_logger("motores_node"), "Failed to set Velocity Control Mode.");
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("motores_node"), "Succeeded to set Velocity Control Mode.");
    }
}

bool Motores::write_velocity(int32_t velocity_left, int32_t velocity_right)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Write Goal Velocity left motor
    dxl_comm_result =
        packetHandler->write4ByteTxRx(
            portHandler,
            MOTOR_LEFT_ID,
            ADDR_GOAL_VELOCITY,
            velocity_left,
            &dxl_error);
    // Write Goal Velocity right motor
    dxl_comm_result =
        packetHandler->write4ByteTxRx(
            portHandler,
            MOTOR_RIGHT_ID,
            ADDR_GOAL_VELOCITY,
            -velocity_right,
            &dxl_error);

}

int32_t Motores::read_position(uint8_t id)
{
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;
    int present_position;

    dxl_comm_result =
        packetHandler->read4ByteTxRx(
            portHandler,
            id,
            ADDR_PRESENT_POSITION,
            reinterpret_cast<uint32_t *>(&present_position),
            &dxl_error);
}

int32_t Motores::read_velocity(uint8_t id)
{
    int dxl_comm_result = COMM_TX_FAIL;
    int present_position;
    uint8_t dxl_error = 0;

    dxl_comm_result =
        packetHandler->read4ByteTxRx(
            portHandler,
            id,
            ADDR_PRESENT_POSITION,
            reinterpret_cast<uint32_t *>(&present_position),
            &dxl_error);

}