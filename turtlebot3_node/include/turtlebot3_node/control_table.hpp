// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Darby Lim

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
  // Table of dynamixel motor XL 430 - W 250
  ControlItem model_number = {0, EEPROM, 2, READ};
  ControlItem model_information = {2, EEPROM, 4, READ};
  ControlItem firmware_version = {6, EEPROM, 1, READ};
  ControlItem id = {7, EEPROM, 1, READ};
  ControlItem baud_rate = {8, EEPROM, 1, READ};
  ControlItem drive_mode = {10, EEPROM, 1, READ_WRITE};
  ControlItem operating_mode = {11, EEPROM, 1, READ_WRITE}; 
  ControlItem protocol_type = {13, EEPROM, 1, READ_WRITE};

  ControlItem motor_torque_enable = {64, RAM, 1, READ_WRITE};
  ControlItem led = {65, RAM, 1, READ_WRITE};
  ControlItem hardware_error_status = {70, RAM, 1, READ};
  ControlItem profile_velocity = {112, RAM, 4, READ_WRITE};
  ControlItem profile_acceleration = {108, RAM, 4, READ_WRITE};
  ControlItem goal_velocity = {104, RAM, 4, READ_WRITE};
  ControlItem goal_position = {116, RAM, 4, READ_WRITE};
  ControlItem goal_pwm = {100, RAM, 2, READ_WRITE};
  ControlItem present_pwm = {124, RAM, 2, READ};
  ControlItem present_load = {126, RAM, 2, READ};
  ControlItem present_velocity = {128, RAM, 4, READ};
  ControlItem present_input_voltage = {144, RAM, 2, READ};

} ControlTable;

const ControlTable extern_control_table;
}  // namespace turtlebot3
}  // namespace robotis

#endif  // TURTLEBOT3_NODE__CONTROL_TABLE_HPP_
