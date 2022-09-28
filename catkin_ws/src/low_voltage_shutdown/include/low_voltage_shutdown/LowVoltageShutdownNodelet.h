#pragma once

#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"
#include "autominy_msgs/msg/voltage.hpp"
#include <dynamic_reconfigure/server.h>
#include <low_voltage_shutdown/LowVoltageShutdownConfig.h>

namespace low_voltage_shutdown {

    class LowVoltageShutdownNodelet : public nodelet::Nodelet {

    public:
        void onInit() override;
        void onVoltage(const autominy_msgs::msg::VoltageConstPtr& msg);
        void onConfig(const LowVoltageShutdownConfig& config, uint32_t level);

    private:
        /// subscriber
        rclcpp::Subscription<>::SharedPtr voltageSubscriber;

        boost::shared_ptr<dynamic_reconfigure::Server<LowVoltageShutdownConfig>> configServer;
        LowVoltageShutdownConfig config;

    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(low_voltage_shutdown::LowVoltageShutdownNodelet, nodelet::Nodelet);
