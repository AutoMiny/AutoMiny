#include "remote_control/httpcommunicator.h"

#include <ros/ros.h>

#include <functional>
#include <string>

namespace remote_control
{
	HttpCommunicator::HttpCommunicator(Backend& backend, std::uint16_t port)
		: m_backend(backend)
		, m_docroot("./src/remote_control/src/templates/httpcommunicator/")
	{
        m_backend.add_image_callback(std::bind(&HttpCommunicator::on_new_image, this, std::placeholders::_1));
        m_server_thread = std::thread(&HttpCommunicator::run_server, this, port);
	}

    HttpCommunicator::~HttpCommunicator()
    {
        m_endpoint.stop_listening();
        if (m_server_thread.joinable())
        {
            m_server_thread.join();
        }
    }

    void HttpCommunicator::on_http(connection_hdl_t hdl) {
        // Upgrade our connection handle to a full connection_ptr
        server_t::connection_ptr con = m_endpoint.get_con_from_hdl(hdl);
    
        std::ifstream file;
        std::string filename = con->get_resource();
        std::string response;
    
        m_endpoint.get_alog().write(websocketpp::log::alevel::app, "http request1: " + filename);
    
        if (filename == "/")
        {
            filename = m_docroot + "index.html";
        }
        else
        {
            filename = m_docroot + filename.substr(1);
        }
        
        m_endpoint.get_alog().write(websocketpp::log::alevel::app, "http request2: " + filename);
    
        file.open(filename.c_str(), std::ios::in);
        if (!file)
        {
            // 404 error
            std::stringstream ss;
        
            ss << "<!doctype html><html><head>"
               << "<title>Error 404 (Resource not found)</title><body>"
               << "<h1>Error 404</h1>"
               << "<p>The requested URL " << filename << " was not found on this server.</p>"
               << "</body></head></html>";
        
            con->set_body(ss.str());
            con->set_status(websocketpp::http::status_code::not_found);
            return;
        }
    
        file.seekg(0, std::ios::end);
        response.reserve(file.tellg());
        file.seekg(0, std::ios::beg);
    
        response.assign((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    
        con->set_body(response);
        con->set_status(websocketpp::http::status_code::ok);
    }

    void HttpCommunicator::on_open(connection_hdl_t hdl)
    {
        ROS_INFO("On Open.");
        m_connections.insert(hdl);
    }

    void HttpCommunicator::on_close(connection_hdl_t hdl)
    {
    	ROS_INFO("On Close.");
        m_connections.erase(hdl);
    }

	void HttpCommunicator::on_message(connection_hdl_t hdl, server_t::message_ptr msg)
	{
        const auto & data_string = msg->get_payload();
		auto speed_string = data_string.substr(0, data_string.find(','));
        auto steering_string = data_string.substr(data_string.find(',') + 1);

        auto speed = std::stof(speed_string);
        auto steering = std::stof(steering_string);

        m_backend.set_speed(static_cast<std::int16_t>((speed * 2.f - 1.f) * 4500));
        m_backend.set_steering(static_cast<std::int16_t>(steering * 180));
	}

    void HttpCommunicator::on_new_image(const image_t& image)
    {
        auto image_size = static_cast<std::uint32_t>(image.size());
        auto speed = m_backend.get_speed();
        auto steering = m_backend.get_steering();

        static std::vector<std::uint8_t> data;
        auto actual_size = image_size + 4;
        if (data.size() < actual_size)
        {
            data.resize(actual_size);
        }
        std::size_t index = 0;

        std::memcpy(&data[0] + index, &speed, 2);
        index += 2;

        std::memcpy(&data[0] + index, &steering, 2);
        index += 2;

        std::memcpy(&data[0] + index, &image[0], image.size());

        for (const auto & connection : m_connections)
        {
            m_endpoint.send(connection, data.data(), actual_size, websocketpp::frame::opcode::binary);
        }
    }

    void HttpCommunicator::run_server(std::uint16_t port)
    {
        m_endpoint.clear_access_channels(websocketpp::log::alevel::all);
        m_endpoint.set_access_channels(websocketpp::log::alevel::access_core);
        m_endpoint.set_access_channels(websocketpp::log::alevel::app);
        m_endpoint.set_error_channels(websocketpp::log::elevel::all);
        m_endpoint.init_asio();
        
        using websocketpp::lib::placeholders::_1;
        using websocketpp::lib::placeholders::_2;
        using websocketpp::lib::bind;

        m_endpoint.set_open_handler(bind(&HttpCommunicator::on_open,this,_1));
        m_endpoint.set_close_handler(bind(&HttpCommunicator::on_close,this,_1));
        m_endpoint.set_http_handler(bind(&HttpCommunicator::on_http,this,_1));
        m_endpoint.set_message_handler(bind(&HttpCommunicator::on_message,this,::_1,::_2));

        m_endpoint.listen(port);
        m_endpoint.start_accept();

        m_endpoint.run();
    }
}
