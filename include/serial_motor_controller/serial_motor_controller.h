#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <exception>
#include <utility>


class serial_motor_controller {
	public:
		serial_motor_controller() {};

		virtual bool send(std::string command) {
			try {
				serial_port.write(command);
			} catch (const std::exception &e) {
				ROS_ERROR("Serial exception!: %s", e.what());
				return false;
			}
			
			return true;
		};

		virtual bool startup() = 0; // return true if startup was successful
		virtual void shutdown() = 0;

		std::pair<double, double> twist_to_wheel_speeds(const geometry_msgs::Twist::ConstPtr &twist) {
			double left_speed = (twist->linear.x - twist->angular.z);
			double right_speed = (twist->linear.x + twist->angular.z);

			return std::make_pair(left_speed, right_speed);
		}

	protected:
		serial::Serial serial_port;
		serial::utils::SerialListener serial_listener;

		ros::Subscriber control_input;
};
