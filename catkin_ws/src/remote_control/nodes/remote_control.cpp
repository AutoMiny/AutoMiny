#include <iostream>

#include <ros/ros.h>

#include <remote_control/backend.h>
#include <remote_control/xboxcommunicator.h>

void emergency_stop(bool emergency_on) {
	std::cout << "emergency stop: " << emergency_on << std::endl;
}

int main(int argc, char **argv)
{
	ROS_INFO("Start remote_control_node.");

	ros::init(argc, argv, "remote_control_node");

	ROS_INFO("Initialize back- and frontend.");
	remote_control::Backend backend;
	
	ROS_INFO("Initialize XBoxCommunicator");
	remote_control::XboxCommunicator xboxCommunicator(backend);

	backend.set_speed(0);
	backend.set_steering(90);
	backend.add_emergency_stop_callback(emergency_stop);

	ros::Rate r(100); // 100 Hz
	while (ros::ok())
	{
		if (xboxCommunicator.is_connected())
		{
			xboxCommunicator.poll();
		}
		else
		{
			xboxCommunicator.connect();
		}
		
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
