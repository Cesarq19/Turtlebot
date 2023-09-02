#ifndef TURTLEBOT3_MOBILITY__VELOCITY_CMD_NODE_HPP_
#define TURTLEBOT3_MOBILITY__VELOCITY_CMD_NODE_HPP_

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

class VelocityCmdNode : public rclcpp::Node
{
    public:
        VelocityCmdNode();
        virtual ~VelocityCmdNode();

    private:
        void setupMotors(uint32_t dxl_id_1, uint8_t dxl_id_2);
        void cmd_vel_callback();
        int32_t linear_velocity_;
        int32_t angular_velocity_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;


}
#endif // TURTLEBOT3_MOBILITY__VELOCITY_CMD_NODE_HPP_