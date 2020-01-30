#include "roboteq_mdc2460.h"

roboteq_mdc2460::roboteq_mdc2460() {
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

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	nh_private.param("device", device_name, std::string("/dev/ttyUSB0"));
	nh_private.param("gear_reduction", gear_reduction, 1.0);
	nh_private.param("has_encoders", has_encoders, false);

	control_input = nh.subscribe("control_vel", 1, &roboteq_mdc2460::control_cb, this);
	// encoder_output = nh.advertise<isc_shared_msgs::EncoderCounts>("encoder_counts", 1000);
}

roboteq_mdc2460::~roboteq_mdc2460() {
	shutdown();
}

bool roboteq_mdc2460::startup() {
	if (serial_port.isOpen()) {
		shutdown();
	}

	// configure the serial port
	serial_port.setPort(device_name);
	serial_port.setBaudrate(115200);
	serial_port.setBytesize(serial::eightbits);
	serial_port.setParity(serial::parity_none);
	serial_port.setStopbits(serial::stopbits_one);

	serial::Timeout t = serial::Timeout::simpleTimeout(10);
	serial_port.setTimeout(t);

	// configure the serial listener
	serial_listener.setChunkSize(64);
	serial_listener.setTokenizer(serial::utils::SerialListener::delimeter_tokenizer("\r\n"));
	serial_listener.setDefaultHandler(boost::bind(&roboteq_mdc2460::receive, this, _1));
	
	// open port and start listening
	try {
		serial_port.open();
		serial_listener.startListening(serial_port);
	} catch (const std::exception &e) {
		ROS_ERROR("Serial exception!: %s", e.what());
		return false;
	}
	
	ROS_INFO("Connected to NextGen Roboteq!");
		
	return true;
}

void roboteq_mdc2460::shutdown() {
	if (serial_port.isOpen()) {
		serial_listener.stopListening();
		serial_port.close();
	}
}

void roboteq_mdc2460::control_cb(const geometry_msgs::Twist::ConstPtr &cmd) {
	std::pair<int, int> wheel_speeds = twist_to_wheel_speeds(cmd);

	std::string left_move_cmd = "!G 1 ";
	std::string right_move_cmd = "!G 2 ";
	
	left_move_cmd += boost::lexical_cast<std::string>(constrain_speed(wheel_speeds.first));
	right_move_cmd += boost::lexical_cast<std::string>(constrain_speed(wheel_speeds.second));

	send(left_move_cmd);
	send(right_move_cmd);
}

void roboteq_mdc2460::receive(std::string response) {
	ROS_INFO("Received from Roboteq: %s", response.c_str());
}
