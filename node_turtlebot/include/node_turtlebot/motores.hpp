#ifndef MOTORES_HPP
#define MOTORES_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "node_turtlebot/odometry.hpp"
#include "node_turtlebot/sensors/joint_state.hpp"
#include "node_turtlebot/sensors/sensors.hpp"



class MotoresNode : public rclcpp::Node {
public:
    MotoresNode();
    int run();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Node::SharedPtr node_handle_;
    std::list<Sensors *> sensors_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_motor_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    int left_present_pos;
    int right_present_pos;
};

#endif  // MOTORES_HPP
