#include <node_turtlebot/sensors/imu.hpp>

#include <utility>

Imu::Imu(
    std::shared_ptr<rclpp::Node> &nh,
    const std::string &imu_topic_name,
    const std::string &mag_topic_name,
    const std::string &frame_id)
    : Sensors(nh, frame_id)
{
    imu_pub_ = nh->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, this->qos_);
    mag_pub_ = nh->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic_name, this->qos_);

    RCLCPP_INFO(nh_->get_logger(), "Succeeded to create imu publisher");
}

void Imu::publish(const rclcpp::Time &now)
{
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();

    imu_msg->header.frame_id = this->frame_id_;
    imu_msg->header.stamp = now;
    
    auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();

    mag_msg->header.frame_id = this->frame_id_;
    mag_msg->header.stamp = now;

    imu_pub_->publish(std::move(imu_msg));
    mag_pub_->publish(std::move(mag_msg));
}