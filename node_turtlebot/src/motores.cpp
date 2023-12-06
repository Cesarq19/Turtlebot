#include "motores.hpp"

PortHandler *portHandler;
PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint32_t goal_velocity = 0;

MotoresNode::MotoresNode() : Node("motores_node") {
    // Publicadores para los motores izquierdo y derecho
    left_motor_publisher_ = this->create_publisher<std_msgs::msg::Float64>("left_motor_cmd", 10);
    right_motor_publisher_ = this->create_publisher<std_msgs::msg::Float64>("right_motor_cmd", 10);

    // Suscriptor al tópico cmd_vel para recibir comandos de velocidad
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotoresNode::cmdVelCallback, this, std::placeholders::_1)
    );
}

void MotoresNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Convertir comandos de velocidad a velocidades de motor y publicar
    std_msgs::msg::Float64 left_motor_cmd;
    std_msgs::msg::Float64 right_motor_cmd;

    // Lógica para convertir comandos de velocidad a comandos de motor
    // ...

    // Publicar comandos de motor
    left_motor_publisher_->publish(left_motor_cmd);
    right_motor_publisher_->publish(right_motor_cmd);
}

void setupMotors()
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

int main(int argc, char** argv) {
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

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoresNode>());
    rclcpp::shutdown();
    
    return 0;
}
