#ifndef VELOCITY_CMD_NODE_HPP_
#define VELOCITY_CMD_NODE_HPP_

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

class VelocityCmdNode : public rclcpp::Node
{
    public:
        VelocityCmdNode();
        virtual ~VelocityCmdNode();

    private:
        void cmd_vel_callback();
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;


}
#endif