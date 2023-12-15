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
// #include <turtlebot3_msgs/msg/sensor_state.hpp>

// include "node_turtlebot/control_table.hpp"
//#include "node_turtlebot/devices/devices.hpp"
//#include "node_turtlebot/devices/motor_power.hpp"
//#include "node_turtlebot/devices/reset.hpp"
//#include "node_turtlebot/devices/sound.hpp"
//#include "node_turtlebot/dynamixel_sdk_wrapper.hpp"
#include "node_turtlebot/odometry.hpp"
//#include "node_turtlebot/sensors/battery_state.hpp"
//#include "node_turtlebot/sensors/imu.hpp"
#include "node_turtlebot/sensors/joint_state.hpp"
//#include "node_turtlebot/sensors/sensor_state.hpp"
#include "node_turtlebot/sensors/sensors.hpp"

// Control table address
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

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
    void setupMotors();
    int run();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_motor_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_motor_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

    int left_present_pos;
    int right_present_pos;
};

#endif  // MOTORES_HPP
