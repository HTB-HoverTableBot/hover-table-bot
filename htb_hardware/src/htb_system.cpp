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


#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "htb_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace htb_hardware
{
hardware_interface::CallbackReturn HtbSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  _clock(RCL_SYSTEM_TIME);

  serial_port_name = DEFAULT_PORT;
  rclcpp::Node demo_node("Nodo");
  std::string my_param =demo_node.get_parameter("serial_port").get_parameter_value().get<std::string>();
  
  if ((port_fd = open(serial_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("HtbSystemHardware"), "Cannot open serial port to hoverboard");
    exit(-1);
  }

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Convert m/s to rad/s
  max_velocity /= wheel_radius;

  low_wrap = ENCODER_LOW_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
  high_wrap = ENCODER_HIGH_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
  last_wheelcountR = last_wheelcountL = 0;
  multR = multL = 0;


  // CONFIGURE THE UART -- connecting to the board
  // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  struct termios options;
  tcgetattr(port_fd, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(port_fd, TCIFLUSH);
  tcsetattr(port_fd, TCSANOW, &options);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // HtbSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HtbSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HtbSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HtbSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HtbSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HtbSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HtbSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HtbSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn HtbSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystemHardware"), "Activating HtbSystemHardware...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("HtbSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn HtbSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("HtbSystemHardware"), "Deactivating HtbSystemHardware...please wait...");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type HtbSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double radius = 0.02;  // radius of the wheels
  double dist_w = 0.1;   // distance between the wheels

  // Update the free-flyer, i.e. the base notation using the classical
  // wheel differentiable kinematics
  double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  base_x_ += base_dx * period.seconds();
  base_y_ += base_dy * period.seconds();
  base_theta_ += base_dtheta * period.seconds();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type htb_hardware::HtbSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("HtbSystemHardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("HtbSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("HtbSystemHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

void HtbSystemHardware::protocol_recv(char byte){

  start_frame = ((uint16_t)(byte) << 8) | (uint8_t)prev_byte;

  // Read the start frame
  if (start_frame == START_FRAME) {
    p = (char*)&msg;
    *p++ = prev_byte;
    *p++ = byte;
    msg_len = 2;
  } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
    // Otherwise just read the message content until the end
    *p++ = byte;
    msg_len++;
  }

  if (msg_len == sizeof(SerialFeedback)) {
    uint16_t checksum = (uint16_t)(
        msg.start ^
        msg.cmd1 ^
        msg.cmd2 ^
        msg.speedR_meas ^
        msg.speedL_meas ^
        msg.wheelR_cnt ^
        msg.wheelL_cnt ^
        msg.batVoltage ^
        msg.boardTemp ^
        msg.cmdLed);

    if (msg.start == START_FRAME && msg.checksum == checksum) {

      double batt_voltage = (double)msg.batVoltage/100.0;
      // voltage_pub.publish(f);

      double board_temp = (double)msg.boardTemp/10.0;
      // temp_pub.publish(f);

      // Convert RPM to RAD/S
      // joints[0].vel.data = direction_correction * (abs(msg.speedL_meas) * 0.10472);
      // joints[1].vel.data = direction_correction * (abs(msg.speedR_meas) * 0.10472);
      // vel_pub[0].publish(joints[0].vel);
      // vel_pub[1].publish(joints[1].vel);

      // Process encoder values and update odometry
      on_encoder_update (msg.wheelR_cnt, msg.wheelL_cnt);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("HtbSystemHardware"), "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
    }
    msg_len = 0;
  }
  prev_byte = byte;

}

void HtbSystemHardware::on_encoder_update(int16_t right, int16_t left)
{
  double posL = 0.0, posR = 0.0;

  // Calculate wheel position in ticks, factoring in encoder wraps
  if (right < low_wrap && last_wheelcountR > high_wrap)
    multR++;
  else if (right > high_wrap && last_wheelcountR < low_wrap)
    multR--;
  posR = right + multR * (ENCODER_MAX - ENCODER_MIN);
  last_wheelcountR = right;

  if (left < low_wrap && last_wheelcountL > high_wrap)
    multL++;
  else if (left > high_wrap && last_wheelcountL < low_wrap)
    multL--;
  posL = left + multL * (ENCODER_MAX - ENCODER_MIN);
  last_wheelcountL = left;

  // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
  // This section accumulates ticks even if board shuts down and is restarted
  static double lastPosL = 0.0, lastPosR = 0.0;
  static double lastPubPosL = 0.0, lastPubPosR = 0.0;
  static bool nodeStartFlag = true;

  // IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
  //(the board seems to often report 1-3 ticks on startup instead of zero)
  // reset the last read ticks to the startup values
  if ((rclcpp::Clock::now() - last_read).toSec() > 0.2 && abs(posL) < 5 && abs(posR) < 5)
  {
    lastPosL = posL;
    lastPosR = posR;
  }
  double posLDiff = 0;
  double posRDiff = 0;

  // if node is just starting keep odom at zeros
  if (nodeStartFlag)
  {
    nodeStartFlag = false;
  }
  else
  {
    posLDiff = posL - lastPosL;
    posRDiff = posR - lastPosR;
  }

  lastPubPosL += posLDiff;
  lastPubPosR += posRDiff;
  lastPosL = posL;
  lastPosR = posR;

  // Convert position in accumulated ticks to position in radians
  // joints[0].pos.data = 2.0*M_PI * lastPubPosL/(double)TICKS_PER_ROTATION;
  // joints[1].pos.data = 2.0*M_PI * lastPubPosR/(double)TICKS_PER_ROTATION;

  // pos_pub[0].publish(joints[0].pos);
  // pos_pub[1].publish(joints[1].pos);
}

}  // namespace htb_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  htb_hardware::HtbSystemHardware, hardware_interface::SystemInterface)