#pragma once

#include <boost/asio.hpp>

#include <remote_control/backend.h>

namespace remote_control
{

class XboxCommunicator
{
public:
	XboxCommunicator(Backend &backend) : m_fd(-1), m_joystick(m_ios), m_backend(backend), m_val_x(0), m_val_y(0) {}

	inline bool is_connected() { return m_fd != -1; }

	inline void poll() { m_ios.poll(); }

	void connect();

protected:
	int m_fd;
	boost::asio::io_service m_ios;
	boost::asio::posix::stream_descriptor m_joystick;
	boost::asio::streambuf m_buffer;
	Backend& m_backend;
	int m_val_x, m_val_y;

	void on_read(const boost::system::error_code& error, std::size_t bytes_transferred);
};

}
