#include "node_turtlebot/diff_drive_controller.hpp"

#include <memory>

DiffDriveController::DiffDriveController()
: Node("diff_drive_controller", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  odometry_ = std::make_unique<Odometry>(
    nh_,
    0.160,  // wheel_separation
    0.033
  );

  RCLCPP_INFO(this->get_logger(), "Run!");
}