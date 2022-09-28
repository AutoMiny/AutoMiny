#include <low_voltage_shutdown/LowVoltageShutdownNodelet.h>

namespace low_voltage_shutdown {

    void LowVoltageShutdownNodelet::onInit() {
        ros::NodeHandle nh = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

        configServer = boost::make_shared<dynamic_reconfigure::Server<LowVoltageShutdownConfig> >(pnh);
        dynamic_reconfigure::Server<LowVoltageShutdownConfig>::CallbackType f;
        f = boost::bind(&LowVoltageShutdownNodelet::onConfig, this, _1, _2);
        configServer->setCallback(f);

        /// Subscriber
        voltageSubscriber = create_subscription<>("voltage", 10, &LowVoltageShutdownNodelet::onVoltage, this);
    }

    void LowVoltageShutdownNodelet::onConfig(const LowVoltageShutdownConfig& config, uint32_t level) {
        this->config = config;
    }

    void LowVoltageShutdownNodelet::onVoltage(const autominy_msgs::msg::VoltageConstPtr& msg) {
        if (msg->value < config.shutdown_voltage) {
            system("sudo shutdown now");
        }
    }
}
