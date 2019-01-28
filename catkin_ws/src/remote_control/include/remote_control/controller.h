#pragma once

#include <cstdint>
#include <autominy_msgs/NormalizedSpeedCommand.h>
#include <autominy_msgs/NormalizedSteeringCommand.h>
#include <ros/ros.h>

namespace remote_control
{

	class Controller
	{
	public:
		Controller();

		/*
		Sets and emits speed message.
		speed < 0: forward
		speed == 0: halt
		speed > 0: backward
		*/
		void set_speed(double speed);

		/*
		Set and emits steering message.
		steering == 1: right
		steering == 0: straight
		steering == -1: left
		*/
		void set_steering(double steering);

	protected:

		// Subscriber handles, if destructed the node stops subscribing
		ros::NodeHandle m_node_handle;
		ros::Publisher m_pub_speed, m_pub_steering;
	};

}
