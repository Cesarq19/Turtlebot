#include "node_turtlebot/core_motores.hpp"
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
    
    Motores::initMotors();

    // Publicadores para los motores izquierdo y derecho (Positions)
    //left_motor_publisher_ = this->create_publisher<std_msgs::msg::Int32>("left_motor_pos", 10);
    //right_motor_publisher_ = this->create_publisher<std_msgs::msg::Int32>("right_motor_pos", 10);

    // Suscriptor al tópico cmd_vel para recibir comandos de velocidad
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotoresNode::cmdVelCallback, this, std::placeholders::_1)
    );
}

void MotoresNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Convertir comandos de velocidad a velocidades de motor y publicar
    //std_msgs::msg::Int32 left_motor_pos;
    //std_msgs::msg::Int32 right_motor_pos;

    int32_t linear_velocity_ = static_cast<int32_t>(msg->linear.x * 100);
    int32_t angular_velocity_ = static_cast<int32_t>(msg->angular.z * 100);

    int32_t velocity_left = linear_velocity_ - angular_velocity_;
    int32_t velocity_right = linear_velocity_ + angular_velocity_;

    Motores::write_velocity(velocity_left, velocity_right);

    //left_motor_pos.data = left_present_pos;
    //right_motor_pos.data = right_present_pos;

    // Publicar comandos de motor
    //left_motor_publisher_->publish(left_motor_pos);
    //right_motor_publisher_->publish(right_motor_pos);
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

    return 0;
}
