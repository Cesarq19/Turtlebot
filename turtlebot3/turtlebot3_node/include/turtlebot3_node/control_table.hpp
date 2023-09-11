#ifndef TURTLEBOT3_NODE__CONTROL_TABLE_HPP_
#define TURTLEBOT3_NODE__CONTROL_TABLE_HPP_

#include <stdlib.h>

namespace robotis
{
    namespace turtlebot3
    {
        constexpr uint8_t EEPROM = 1;
        constexpr uint8_t RAM = 2;

        constexpr uint8_t READ = 1;
        constexpr uint8_t READ_WRITE = 3;

        typedef struct
        {
            uint16_t addr;
            uint8_t memory;
            uint16_t length;
            uint8_t rw;
        } ControlItem;

        typedef struct
        {
            ControlItem model_number = {0, EEPROM, 2, READ};
            ControlItem model_information = {2, EEPROM, 4, READ};
            ControlItem firmware_version = {6, EEPROM, 1, READ};
            ControlItem id = {7, EEPROM, 1, READ};
            ControlItem baud_rate = {8, EEPROM, 1, READ};
            ControlItem drive_mode = {10, EEPROM, 1, READ_WRITE};

            ControlItem torque_enable = {64, RAM, 1, READ_WRITE};
            ControlItem led = {65, RAM, 1, READ_WRITE};
            ControlItem hw_error = {70, RAM, 1, READ};

            ControlItem present_pwm = {124, RAM, 2, READ};
            ControlItem present_velocity = {128, RAM, 4, READ};
            ControlItem present_position = {132, RAM, 4, READ};
            ControlItem present_input_voltage = {144, RAM, 2, READ};

            ControlItem profile_acceleration = {108, RAM, 4, READ_WRITE};
            ControlItem profile_velocity = {112, RAM, 4, READ_WRITE};

        } ControlTable;

        const ControlTable extern_control_table;
    }
}
#endif
