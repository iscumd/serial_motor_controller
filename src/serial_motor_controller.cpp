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

#include <serial_motor_controller/serial_motor_controller.hpp>

namespace serial_motor_controller
{

SerialMotorController::SerialMotorController(rclcpp::NodeOptions options)
    : Node("serial_motor_controller", options)
{
    command_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&SerialMotorController::cmd_vel_callback, this, std::placeholders::_1));
}

void SerialMotorController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    last_command_vel = *msg;
}

bool SerialMotorController::connect(const std::string port)
{
    if(port.empty())
    {
        return false;
    }
    else if(serial_port.isOpen())
    {
        return false;
    }
    else
    {
        serial_port.setPort(port);
        serial_port.setBaudrate(9600);
        serial_port.setParity(serial::parity_even);
        serial_port.setStopbits(serial::stopbits_one);
        serial_port.setBytesize(serial::sevenbits);
        serial::Timeout time = serial::Timeout::simpleTimeout(10);
        serial_port.setTimeout(time);

        serial_port.open();
        return true;
    }
}

bool SerialMotorController::send_command(const std::string& command)
{
    try
    {
        serial_port.write(command);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Serial exception!: %s", e.what());
        return false;
    }
    return true;
}

}  // namespace serial_motor_controller