#include "low_voltage_shutdown/LowVoltageShutdownNodelet.h"

namespace low_voltage_shutdown {

    LowVoltageShutdownNodelet::LowVoltageShutdownNodelet(const rclcpp::NodeOptions &opts) : rclcpp::Node("low_voltage_shutdown", opts)  {
        shutdownVoltage = declare_parameter<double>("shutdown_voltage", 13.0);
        voltageSubscriber = create_subscription<autominy_msgs::msg::Voltage>("/sensors/voltage", 10, std::bind(&LowVoltageShutdownNodelet::onVoltage, this, std::placeholders::_1));
    }

    void LowVoltageShutdownNodelet::onVoltage(const autominy_msgs::msg::Voltage::ConstSharedPtr &msg) {
        if (msg->value < shutdownVoltage) {
            system("sudo shutdown now");
        }
    }
}
