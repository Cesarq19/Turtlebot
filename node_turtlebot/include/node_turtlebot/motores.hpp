#ifndef MOTORES_HPP
#define MOTORES_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table address
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128

// Protocol version
#define PROTOCOL_VERSION 2.0 

// Default setting
#define MOTOR_LEFT_ID 1
#define MOTOR_RIGHT_ID 0
#define BAUDRATE 57600                                                                                // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

class MotoresNode : public rclcpp::Node {
public:
    MotoresNode();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_motor_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
};

#endif  // MOTORES_HPP
