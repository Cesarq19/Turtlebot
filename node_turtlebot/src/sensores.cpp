#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "node_turtlebot/sensores.hpp"


SensoresNode::SensoresNode() : Node("sensores_node") {
    // Publicadores
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // Temporizador para simular la publicación de datos de sensores
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SensoresNode::publishSensorData, this));

    // Inicializar el estado del joint
    joint_state_.name = {"left_wheel_joint", "right_wheel_joint"};
    joint_state_.position.resize(2, 0.0);
    joint_state_.velocity.resize(2, 0.0);

    // Inicializar la odometría
    odometry_msg_.header.frame_id = "odom";
    odometry_msg_.child_frame_id = "base_link";
    odometry_msg_.pose.pose.position.x = 0.0;
    odometry_msg_.pose.pose.position.y = 0.0;
    odometry_msg_.pose.pose.position.z = 0.0;
    odometry_msg_.pose.pose.orientation.x = 0.0;
    odometry_msg_.pose.pose.orientation.y = 0.0;
    odometry_msg_.pose.pose.orientation.z = 0.0;
    odometry_msg_.pose.pose.orientation.w = 1.0;
    odometry_msg_.twist.twist.linear.x = 0.0;
    odometry_msg_.twist.twist.linear.y = 0.0;
    odometry_msg_.twist.twist.angular.z = 0.0;

    joint_state_publisher_->publish(joint_state_);

    odom_publisher_->publish(odometry_msg_);

}

void SensoresNode::run(){
    publishSensorData();
}

void SensoresNode::publishSensorData() {
    // Simular datos de sensores y actualizar el estado del joint
    joint_state_.position[0] += 0.01;  // Izquierda
    joint_state_.position[1] += 0.02;  // Derecha

    // Publicar el estado del joint
    joint_state_publisher_->publish(joint_state_);

    // Simular datos de odometría y actualizar la posición
    odometry_msg_.header.stamp = this->now();
    odometry_msg_.pose.pose.position.x += 0.01;
    odometry_msg_.pose.pose.position.y += 0.02;
    odometry_msg_.pose.pose.orientation.z += 0.01;

    // Publicar la odometría
    odom_publisher_->publish(odometry_msg_);
}
