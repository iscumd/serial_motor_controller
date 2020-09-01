#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <exception>
#include <utility>

namespace serial_motor_controller
{
/*
	params:
		string device -> the name of the serial port to communicate on (default: /dev/ttyUSB0)
		double gear_reduction -> what the gear reduction is where the motor RPM is being read (default: 1.0)
		bool has_encoders -> whether encoders are being used (default: false)

	subscriptions:
		geometry_msgs/Twist control_vel -> input topic for motor commands

	publications:
		isc_shared_msgs/EncoderCounts encoder_counts ->	output topic for encoder rotation counts
*/
class serial_motor_controller
{
protected:
  serial::Serial serial_port;
  serial::utils::SerialListener serial_listener;

  serial_motor_controller() {}
  virtual ~serial_motor_controller() {}

  virtual bool startup() = 0; // return true if startup was successful
  virtual void shutdown() = 0;

  virtual bool send(const std::string& command)
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
  };

  int constrain_speed(const int& speed, const int& max_speed)
  {
    return (speed > 0 ? 1 : -1) * std::min(std::abs(speed), max_speed);
  }

  std::pair<double, double> twist_to_wheel_speeds(const geometry_msgs::Twist::ConstPtr& twist)
  {
    double left_speed = (twist->linear.x - twist->angular.z);
    double right_speed = (twist->linear.x + twist->angular.z);
    return std::make_pair(left_speed, right_speed);
  }
};
}  // namespace serial_motor_controller
