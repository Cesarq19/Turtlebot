#ifndef CORE_MOTORES_HPP
#define CORE_MOTORES_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <rclcpp/rclcpp.hpp>
#include "estructuras.h"

using namespace dynamixel;

// Control table address
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_VELOCITY 104
#define ADDR_PRESENT_VELOCITY 128
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0 

// Default setting
#define MOTOR_LEFT_ID 0
#define MOTOR_RIGHT_ID 1
#define BAUDRATE 57600                                                                                // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.4:1.0-port0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

class Motores
{
    public:
        static void initMotors();
        static bool write_velocity(int32_t velocity_left, int32_t velocity_right);
        static int32_t read_position(uint8_t id);
        static int32_t read_velocity(uint8_t id);
    private:
        inline static PortHandler *portHandler;
        inline static PacketHandler *packetHandler;


};

#endif  // CORE_MOTORES_HPP