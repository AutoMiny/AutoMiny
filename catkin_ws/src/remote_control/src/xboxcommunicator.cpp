#include "remote_control/xboxcommunicator.h"

#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include <cmath>
#include <cctype>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/joystick.h>

namespace remote_control
{

	void XboxCommunicator::on_read(const boost::system::error_code& error, std::size_t bytes_transferred)
	{
		if (error == boost::system::errc::no_such_device)
		{
			ROS_INFO("Xbox controller was disconnected");
			m_joystick.cancel();
			m_joystick.close();
			m_buffer.consume(m_buffer.size());
			m_fd = -1;
			return;
		}
		else if (error)
		{
			ROS_INFO("Internal error: %s", error.message().c_str());
			return;
		}

		if (bytes_transferred == sizeof(js_event))
		{
			auto jse = boost::asio::buffer_cast<const js_event*>(m_buffer.data());

			if ((jse->type & ~JS_EVENT_INIT) == JS_EVENT_AXIS)
			{
				const int threshold = 14000;
				
				int value = jse->value;
				if (value < -threshold)
				{
					value += threshold;
				}
				else if (value > threshold) 
				{
					value -= threshold;
				}
				else
				{
					value = 0;
				}
				
				if (jse->number == 3)
				{
					m_val_x = value;
				}
				else if (jse->number == 1)
				{
					m_val_y = value;
				}

				float x = static_cast<float>(m_val_x) / (32767 - threshold);
				float sign_x = (x>=0)? 1.0 : -1.0;
				float y = static_cast<float>(m_val_y) / (32767 - threshold);
				float sign_y = (y>=0)? 1.0 : -1.0;

					
				m_backend.set_speed(-0.35 * std::pow(y, 2) * sign_y);
				m_backend.set_steering(-std::pow(x,3));
			}

			m_buffer.consume(bytes_transferred);
		}

		boost::asio::async_read(
			m_joystick, m_buffer, boost::asio::transfer_exactly(sizeof(js_event)),
			boost::bind(&XboxCommunicator::on_read, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}

	// http://stackoverflow.com/questions/9743485/natural-sort-of-directory-filenames-in-c#answer-9745132
	bool compareNat(const std::string& a, const std::string& b)
	{
	    if (a.empty())
	    {
	        return true;
	    }
	    if (b.empty())
	    {
	        return false;
	    }
	    if (std::isdigit(a[0]) && !std::isdigit(b[0]))
	    {
	        return true;
	    }
	    if (!std::isdigit(a[0]) && std::isdigit(b[0]))
	    {
	        return false;
	    }
	    if (!std::isdigit(a[0]) && !std::isdigit(b[0]))
	    {
	        if (std::toupper(a[0]) == std::toupper(b[0]))
	        {
	            return compareNat(a.substr(1), b.substr(1));
	        }
	        return (std::toupper(a[0]) < std::toupper(b[0]));
	    }

	    // Both strings begin with digit --> parse both numbers
	    std::istringstream issa(a);
	    std::istringstream issb(b);
	    int ia, ib;
	    issa >> ia;
	    issb >> ib;
	    if (ia != ib)
	    {
	        return ia < ib;
	    }

	    // Numbers are the same --> remove numbers and recurse
	    std::string anew, bnew;
	    std::getline(issa, anew);
	    std::getline(issb, bnew);
	    return compareNat(anew, bnew);
	}
	bool compareNat_tuple(const std::tuple<int, std::string, std::string>& a, const std::tuple<int, std::string, std::string>& b)
	{
		return compareNat(std::get<1>(a), std::get<1>(b));
	}

	void XboxCommunicator::connect()
	{
		if (is_connected())
		{
			return;
		}

		// collect all xbox controllers and choose the first one
		std::vector<std::tuple<int, std::string, std::string>> devices;
		boost::regex js_regex("\\/dev\\/input\\/js([0-9]+)");
		for(const auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator("/dev/input"), {}))
		{
			const std::string& device = entry.path().string();
			if (boost::regex_match(device, js_regex) && boost::filesystem::status(entry.path()).type() == boost::filesystem::file_type::character_file) 
			{
				int fd = open(device.c_str(), O_RDONLY);
				if (fd == -1)
				{
					continue;
				}

				char temp_model_name[50 + 1] = {0};
				ioctl(fd, JSIOCGNAME(50), temp_model_name);
				std::string model_name(temp_model_name);

				if (model_name != "Xbox 360 Wireless Receiver" && model_name.substr(0, 23) != "Microsoft X-Box One pad")
				{
					close(fd);
				} 
				else 
				{
					devices.emplace_back(fd, device, model_name);
				}
			}
		}

		std::sort(devices.begin(), devices.end(), compareNat_tuple);

		if (devices.size() >= 1)
		{
			std::string device;
			std::string model_name;
			std::tie(m_fd, device, model_name) = devices[0];

			ROS_INFO("Xbox controller \"%s\" is now connected", model_name.c_str());

			m_ios.reset(); // without this command handle_read isn't called after a reconnect
			m_joystick.assign(m_fd);

			boost::asio::async_read(
				m_joystick, m_buffer, boost::asio::transfer_exactly(sizeof(js_event)),
				boost::bind(&XboxCommunicator::on_read, this,
					boost::asio::placeholders::error,
					boost::asio::placeholders::bytes_transferred));

			for (auto it = ++devices.begin(); it != devices.end(); ++it)
			{
				close(std::get<0>(*it));
			}
		}
	}

}
