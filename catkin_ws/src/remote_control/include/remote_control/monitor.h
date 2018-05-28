#pragma once

#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/CompressedImage.h>

namespace remote_control
{
	using image_t = std::vector<std::uint8_t>;
	using image_callback_t = std::function<void(const image_t&)>;

	class Monitor
	{
	public:
		Monitor();

		// Set those callbacks to get the values
		void set_emergency_stop_callback(const std::function<void(bool)>& callback);
		void set_image_callback(const image_callback_t& callback);

	protected:
		// Callbacks called inside ROS framework
		void emergency_stop_callback(std_msgs::Bool emergency_on);
		void image_callback(const sensor_msgs::CompressedImage& image);

		ros::NodeHandle m_node_handle;
		ros::Subscriber m_sub_emergency_stop, m_sub_image;
		std::function<void(bool)> m_cb_emergency_stop;
		image_callback_t m_cb_image;
	};

}
