#ifndef MOTORES_HPP
#define MOTORES_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MotoresNode : public rclcpp::Node {
public:
    MotoresNode();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

#endif  // MOTORES_HPP
