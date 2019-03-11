#include <remote_control/controller.h>
#include <memory>

namespace remote_control 
{

	Controller::Controller()
		: m_pub_speed(m_node_handle.advertise<autominy_msgs::NormalizedSpeedCommand>("control/normalized_wanted_speed", 1))
		, m_pub_steering(m_node_handle.advertise<autominy_msgs::NormalizedSteeringCommand>("control/normalized_wanted_steering", 1))
	{

	}

	void Controller::set_speed(double speed)
	{
        autominy_msgs::NormalizedSpeedCommand msg;
		msg.value = static_cast<float>(speed);
		msg.header.stamp = ros::Time::now();
		m_pub_speed.publish(msg);
	}

	void Controller::set_steering(double steering)
	{
        autominy_msgs::NormalizedSteeringCommand msg;
		msg.value = static_cast<float>(steering);
        msg.header.stamp = ros::Time::now();
        m_pub_steering.publish(msg);
	}

} // Namespace remote_control
