#ifndef SENSORES_HPP
#define SENSORES_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>

class SensoresNode : public rclcpp::Node {
public:
    SensoresNode();

    void run();

private:
    void publishSensorData();

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_;
    nav_msgs::msg::Odometry odometry_msg_;
};

#endif  // SENSORES_HPP
