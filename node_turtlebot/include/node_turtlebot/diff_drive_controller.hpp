#ifndef TURTLEBOT3_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define TURTLEBOT3_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "node_turtlebot/odometry.hpp"

class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController();
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};

#endif  // TURTLEBOT3_NODE__DIFF_DRIVE_CONTROLLER_HPP_