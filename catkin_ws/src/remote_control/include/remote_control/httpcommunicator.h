#pragma once

#include <fstream>
#include <iostream>
#include <set>
#include <streambuf>
#include <string>
#include <thread>

#include "websocketpp/config/asio_no_tls.hpp"
#include "websocketpp/server.hpp"

#include <remote_control/backend.h>

namespace remote_control
{

    using connection_hdl_t = websocketpp::connection_hdl;
    using server_t = websocketpp::server<websocketpp::config::asio>;
	using cont_list_t = std::set<connection_hdl_t, std::owner_less<connection_hdl_t>>;

	class HttpCommunicator
	{		
	public:
		HttpCommunicator(Backend& backend, std::uint16_t port);
		~HttpCommunicator();

	protected:
		void on_http(connection_hdl_t hdl);		
	    void on_open(connection_hdl_t hdl);
	    void on_close(connection_hdl_t hdl);

	    void on_message(connection_hdl_t hdl, server_t::message_ptr msg);
		void on_new_image(const image_t& image);

		void run_server(std::uint16_t port);

		Backend& m_backend;
		server_t m_endpoint;
		cont_list_t m_connections;
		server_t::timer_ptr m_timer;
    
    	std::thread m_server_thread;
	    std::string m_docroot;
	};

}