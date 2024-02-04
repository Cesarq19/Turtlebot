#include "node_turtlebot/core_motores.hpp"
#include "node_turtlebot/motores.hpp"

#include <memory>
#include <string>

using namespace dynamixel;
using namespace std::chrono_literals;

PortHandler *portHandler;
PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result = COMM_TX_FAIL;
uint32_t goal_velocity = 0;

MotoresNode::MotoresNode() : Node("motores_node", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    RCLCPP_INFO(get_logger(), "Init TurtleBot3 Node Main");
    
    Motores::initMotors();

    // Suscriptor al tópico cmd_vel para recibir comandos de velocidad
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&MotoresNode::cmdVelCallback, this, std::placeholders::_1)
    );

    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    sensors_.push_back(new JointState(node_handle_, "joint_states", "base_link"));

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() -> void
        {
            rclcpp::Time now = this->now();

            for (const auto & sensor : sensors_) {
                sensor->publish(now);
            }
        }
    );
}

void MotoresNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    int32_t linear_velocity_ = static_cast<int32_t>(msg->linear.x * 100);
    int32_t angular_velocity_ = static_cast<int32_t>(msg->angular.z * 100);

    int32_t velocity_left = linear_velocity_ - angular_velocity_;
    int32_t velocity_right = linear_velocity_ + angular_velocity_;

    Motores::write_velocity(velocity_left, velocity_right);
}

