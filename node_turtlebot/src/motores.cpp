#include "motores.hpp"

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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotoresNode>());
    rclcpp::shutdown();
    return 0;
}
