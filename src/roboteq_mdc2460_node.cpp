#include <ros/ros.h>
#include "roboteq_mdc2460.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "roboteq_mdc2460_node");
	
	roboteq_mdc2460 roboteq;
	
	ros::Rate rt(1);

	while (!roboteq.startup() && ros::ok()) {
		rt.sleep();
		ROS_INFO("Attempting to connect to roboteq!");
	}

	ros::spin();

	return 0;
}
