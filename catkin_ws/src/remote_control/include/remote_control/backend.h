#pragma once

#include <cstdint>
#include <functional>

#include "remote_control/monitor.h"
#include "remote_control/controller.h"

namespace remote_control
{

	using emergency_stop_callback_t = std::function<void(bool)>;

	class Backend
	{
	public:
		Backend()
		 : m_set_speed(0)
		 , m_set_steering(0)
		 , m_is_emergency(false)
		{
			m_monitor.set_emergency_stop_callback(std::bind(&Backend::on_emergency_stop, this, std::placeholders::_1));
			m_monitor.set_image_callback(std::bind(&Backend::on_new_image, this, std::placeholders::_1));
		}

		Backend(const Backend&) = delete;
		Backend& operator=(const Backend&) = delete;

		void add_emergency_stop_callback(const emergency_stop_callback_t& emergency_stop_cb)
		{
			m_emergency_stop_callbacks.push_back(emergency_stop_cb);
		}

		void add_image_callback(const image_callback_t& image_cb)
		{
			m_image_callbacks.push_back(image_cb);
		}

		void set_speed(double speed)
		{
			if (m_is_emergency) return;
			m_set_speed = speed;
			m_controller.set_speed(speed);
		}

		void set_steering(double steering)
		{
			if (m_is_emergency) return;
			m_set_steering = steering;
			m_controller.set_steering(steering);
		}

		double get_speed() const
		{
			return m_set_speed;
		}

		double get_steering() const
		{
			return m_set_steering;
		}

	protected:
		void on_emergency_stop(bool emergency_on)
		{
			set_speed(0);
			for(const auto& callback : m_emergency_stop_callbacks)
			{
				callback(emergency_on);
			}
		}

		void on_new_image(const image_t& image) const
		{
			for(const auto& callback : m_image_callbacks)
			{
				callback(image);
			}
		}

		Monitor m_monitor;
		Controller m_controller;
		std::vector<emergency_stop_callback_t> m_emergency_stop_callbacks;
		std::vector<image_callback_t> m_image_callbacks;
		double m_set_speed, m_set_steering;
		bool m_is_emergency;
	};

}
