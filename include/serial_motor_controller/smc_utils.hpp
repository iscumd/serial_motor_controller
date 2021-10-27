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

#include <math.h>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <utility>

namespace serial_motor_controller
{
namespace utils
{

int constrain_speed(const int& speed, const int& max_speed)
{
    return (speed > 0 ? 1 : -1) * std::min(std::abs(speed), max_speed);
}

std::pair<double, double> twist_to_wheel_speeds(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    double left_speed = (msg->linear.x - msg->angular.z);
    double right_speed = (msg->linear.x + msg->angular.z);
    return std::make_pair(left_speed, right_speed);
}

}  // namespace utils
}  // namespace serial_motor_controller