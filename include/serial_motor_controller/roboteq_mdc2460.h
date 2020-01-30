#include "roboteq_motor_controller.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <sstream>

#include <isc_shared_msgs/EncoderCounts.h>

#define SIGN(x) (x > 0 ? 1 : -1)

class roboteq_mdc2460 : public serial_motor_controller {
	public:
		roboteq_mdc2460();
		~roboteq_mdc2460();

		bool send(std::string cmd) {
			cmd += "\r";
			return roboteq_motor_controller::send(cmd);
		};

		bool startup();
		void shutdown();

		void control_cb(const geometry_msgs::Twist::ConstPtr &cmd);

		void receive(std::string response);

		int constrain_speed(int speed) { return SIGN(speed) * std::max(std::abs(speed), 1000); };
		
	private:
		std::string device_name;
		double gear_reduction;
		bool has_encoders;

		std::stringstream msg_builder;

		ros::Publisher encoder_output;
};
