#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/msg/twist.hpp>


class ControlNode : public rclcpp::Node
{
public:
	ControlNode();
	virtual ~ControlNode();

private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
}

