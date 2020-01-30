#include <ros/ros.h>

#include <string>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <utility>
#include <pair>


class roboteq_motor_controller {
	public:
		roboteq_motor_controller();
		~roboteq_motor_controller();

		virtual bool send_command(std::string command) {
			try {
				serial_port.write(command);
			} catch (const serial::PortNotOpenedException &e) {
				// error, port isn't opened
				ROS_ERROR("Serial port was not opened!: %s", e.what());
				return false;
			} catch (const serial::IOException &e) {
				// something went wrong writing, warn the user
				ROS_WARN("Serial IO error!: %s", e.what());
				return false;
			} catch (const serial::Exception &e) {
				// idfk
				ROS_ERROR("Serial exception!: %s", e.what());
				return false;
			}
			
			return true;
		};

		virtual bool startup() 	= 0; // return true if startup was successful
		virtual bool shutdown() = 0;
		virtual bool reset() 	= 0;

		std::pair<double, double> twist_to_wheel_speeds(const geometry_msgs::Twist &twist) {
			double left_speed = (twist.linear.x - twist.angular.z);
			double right_speed = (twist.linear.x + twist.angular.z);

			return std::make_pair(left_speed, right_speed);
		}

	private:
		serial::Serial roboteq_port;
		serial::utils::SerialListener roboteq_listener;

		ros::Subscriber control_input;
};
