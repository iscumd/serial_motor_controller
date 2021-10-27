// MIT License
//
// Copyright (c) 2021 Intelligent Systems Club
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <memory>
#include <exception>

namespace serial_motor_controller
{
class SerialMotorController : public rclcpp::Node
{
protected:
  serial::Serial serial_port;
  serial::utils::SerialListener serial_listener;
  geometry_msgs::msg::Twist last_command_vel;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_vel_sub;

  explicit SerialMotorController(rclcpp::NodeOptions options);
  ~SerialMotorController();

  virtual bool connect(const std::string& port);
  virtual void disconnect();

  virtual void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  virtual bool send_command(const std::string& command);
};
}  // namespace serial_motor_controller
