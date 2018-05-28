#include <remote_control/monitor.h>

namespace remote_control
{

	Monitor::Monitor()
		: m_sub_emergency_stop(m_node_handle.subscribe("manual_control/emergency_stop", 1, &Monitor::emergency_stop_callback, this))
		, m_sub_image(m_node_handle.subscribe("app/camera/rgb/image_raw/compressed", 1, &Monitor::image_callback, this))
	{

	}

	void Monitor::set_emergency_stop_callback(const std::function<void(bool)>& callback)
	{
		m_cb_emergency_stop = callback;
	}

	void Monitor::emergency_stop_callback(std_msgs::Bool emergency_on)
	{
		if (m_cb_emergency_stop) m_cb_emergency_stop(emergency_on.data);
	}

	void Monitor::set_image_callback(const image_callback_t& callback)
	{
		m_cb_image = callback;
	}

	void Monitor::image_callback(const sensor_msgs::CompressedImage& image)
	{
		if (m_cb_image)
		{
			m_cb_image(image.data);
		}
	}

}
