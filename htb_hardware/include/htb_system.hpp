// Copyright 2023 Martin Nievas
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

#ifndef HTB_HARDWARE__HTB_SYSTEM_HPP_
#define HTB_HARDWARE__HTB_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <termios.h>
#include <fcntl.h> // Contains file controls like O_RDWR

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"
#include "config.hpp"
#include "protocol.hpp"
#include <rclcpp/rclcpp.hpp>

namespace htb_hardware
{
class HtbSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HtbSystemHardware);

  HTB_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  HTB_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HTB_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HTB_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  HTB_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  HTB_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  HTB_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  HTB_HARDWARE_PUBLIC
  void protocol_recv(char byte);

  HTB_HARDWARE_PUBLIC
  void on_encoder_update (int16_t right, int16_t left);

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;

  // Driver variables
  double wheel_radius;
  double max_velocity = 0.0;
  int direction_correction = 1;
  int inverted = 1;
  std::string serial_port_name;
  rclcpp::Time last_read;
  rclcpp::Clock _clock;
  // Last known encoder values
  int16_t last_wheelcountR;
  int16_t last_wheelcountL;
  // Count of full encoder wraps
  int multR;
  int multL;
  // Thresholds for calculating the wrap
  int low_wrap;
  int high_wrap;

  // Hoverboard protocol
  int port_fd;
  int msg_len = 0;
  char prev_byte = 0;
  uint16_t start_frame = 0;
  char* p;
  SerialFeedback msg, prev_msg;
};

}  // namespace htb_hardware

#endif  // HTB_HARDWARE__HTB_SYSTEM_HPP_