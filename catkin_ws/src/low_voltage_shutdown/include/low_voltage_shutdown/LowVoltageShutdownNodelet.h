#pragma once

#include "rclcpp/rclcpp.hpp"
#include "autominy_msgs/msg/voltage.hpp"

namespace low_voltage_shutdown {

    class LowVoltageShutdownNodelet : public rclcpp::Node {

    public:
        LowVoltageShutdownNodelet(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions());

        void onVoltage(const autominy_msgs::msg::Voltage::ConstSharedPtr &msg);

    private:
        /// subscriber
        rclcpp::Subscription<autominy_msgs::msg::Voltage>::SharedPtr voltageSubscriber;

        double shutdownVoltage;
    };
}
