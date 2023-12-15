#include "node_turtlebot/motores.hpp"

#include <memory>
#include <string>

using namespace dynamixel;
using namespace std::chrono_literals;

PortHandler *portHandler;
PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint32_t goal_velocity = 0;

MotoresNode::MotoresNode() : Node("motores_node") {
    RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
    // Publicadores para los motores izquierdo y derecho (Positions)
    left_motor_publisher_ = this->create_publisher<std_msgs::msg::Int32>("left_motor_pos", 10);
    right_motor_publisher_ = this->create_publisher<std_msgs::msg::Int32>("right_motor_pos", 10);

    // Suscriptor al tópico cmd_vel para recibir comandos de velocidad
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotoresNode::cmdVelCallback, this, std::placeholders::_1)
    );
}

void MotoresNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Convertir comandos de velocidad a velocidades de motor y publicar
    std_msgs::msg::Int32 left_motor_pos;
    std_msgs::msg::Int32 right_motor_pos;


    // Lógica para convertir comandos de velocidad a comandos de motor
    uint8_t dxl_error = 0;

    int32_t linear_velocity_ = static_cast<int32_t>(msg->linear.x * 100);
    int32_t angular_velocity_ = static_cast<int32_t>(msg->angular.z * 100);

    int32_t velocity_left = linear_velocity_ - angular_velocity_;
    int32_t velocity_right = linear_velocity_ + angular_velocity_;

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

    RCLCPP_INFO(
                this->get_logger(),
                "left_vel: %d right_vel: %d", velocity_left, velocity_right);

    // Read present position left motor
    dxl_comm_result =
        packetHandler->read4ByteTxRx(
            portHandler,
            MOTOR_LEFT_ID,
            ADDR_PRESENT_POSITION,
            reinterpret_cast<uint32_t *>(&left_present_pos),
            &dxl_error);
    // Read present position right motor
    dxl_comm_result =
        packetHandler->read4ByteTxRx(
            portHandler,
            MOTOR_RIGHT_ID,
            ADDR_PRESENT_POSITION,
            reinterpret_cast<uint32_t *>(&right_present_pos),
            &dxl_error);

    left_motor_pos.data = left_present_pos;
    right_motor_pos.data = right_present_pos;

    // Publicar comandos de motor
    left_motor_publisher_->publish(left_motor_pos);
    right_motor_publisher_->publish(right_motor_pos);
}

void MotoresNode::setupMotors()
{
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

int MotoresNode::run() {
    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motores_node"), "Failed to open the port!");
        return -1;
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
        return -1;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("motores_node"), "Succeeded to set the baudrate.");
    }

    setupMotors();
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<MotoresNode>());
    // rclcpp::shutdown();

    return 0;
}

// int main(int argc, char** argv) {
//     portHandler = PortHandler::getPortHandler(DEVICE_NAME);
//     packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//     // Open Serial Port
//     dxl_comm_result = portHandler->openPort();
//     if (dxl_comm_result == false)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("motores_node"), "Failed to open the port!");
//         return -1;
//     }
//     else
//     {
//         RCLCPP_INFO(rclcpp::get_logger("motores_node"), "Succeeded to open the port.");
//     }

//     // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
//     dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
//     if (dxl_comm_result == false)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("motores_node"), "Failed to set the baudrate!");
//         return -1;
//     }
//     else
//     {
//         RCLCPP_INFO(rclcpp::get_logger("motores_node"), "Succeeded to set the baudrate.");
//     }

//     setupMotors();
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MotoresNode>());
//     rclcpp::shutdown();

//     return 0;
// }
