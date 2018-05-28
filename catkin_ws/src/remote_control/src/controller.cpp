#include <remote_control/controller.h>
#include <memory>

namespace remote_control 
{

	Controller::Controller()
		: m_pub_speed(m_node_handle.advertise<std_msgs::Int16>(m_node_handle.resolveName("manual_control/speed"), 1))
		, m_pub_steering(m_node_handle.advertise<std_msgs::UInt8>(m_node_handle.resolveName("/steering"), 1))
	{

	}

	void Controller::set_speed(std::int16_t speed)
	{
		std_msgs::Int16 msg;
		msg.data = speed;
		m_pub_speed.publish(msg);
	}

	void Controller::set_steering(std::int16_t steering)
	{
		std_msgs::UInt8 msg;
		msg.data = steering;
		m_pub_steering.publish(msg);
	}

} // Namespace remote_control
