#pragma once

#include <cstdint>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
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
		void set_speed(std::int16_t speed);

		/*
		Set and emits steering message.
		steering == 0: right
		steering == 90: straight
		steering == 180: left
		*/
		void set_steering(std::int16_t steering);

	protected:

		// Subscriber handles, if destructed the node stops subscribing
		ros::NodeHandle m_node_handle;
		ros::Publisher m_pub_speed, m_pub_steering;
	};

}
