#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string usb_port = "platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0";

}