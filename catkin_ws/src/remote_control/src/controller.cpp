#include <remote_control/controller.h>
#include <memory>

namespace remote_control 
{

	Controller::Controller()
		: m_pub_speed(m_node_handle.advertise<std_msgs::Float64>("control/normalized_wanted_speed", 1))
		, m_pub_steering(m_node_handle.advertise<std_msgs::Float64>("control/normalized_wanted_steering", 1))
	{

	}

	void Controller::set_speed(double speed)
	{
		std_msgs::Float64 msg;
		msg.data = speed;
		m_pub_speed.publish(msg);
	}

	void Controller::set_steering(double steering)
	{
		std_msgs::Float64 msg;
		msg.data = steering;
		m_pub_steering.publish(msg);
	}

} // Namespace remote_control
