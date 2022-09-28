#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"

#include <dynamic_reconfigure/server.h>
#include <emergency_stop/EmergencyStopFwd.h>
#include <emergency_stop/EmergencyStopConfig.h>
#include <emergency_stop/EmergencyStop.h>
#include <sensor_msgs/LaserScan.h>
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include "autominy_msgs/msg/speed.hpp"

namespace emergency_stop {

/** EmergencyStop nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class EmergencyStopNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        EmergencyStopNodelet() = default;

        /** Destructor.
         */
        ~EmergencyStopNodelet() override {}

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        virtual void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            emergencyStop = std::make_shared<EmergencyStop>();

            speedPublisher = pcreate_publisher<autominy_msgs::msg::SpeedPWMCommand>("speed", 1);
            scanSubscriber = create_subscription<>("scan", 1, &EmergencyStopNodelet::onScan, this, ros::TransportHints().tcpNoDelay());
            wantedSpeedSubscriber = create_subscription<>("wanted_speed", 1, &EmergencyStopNodelet::onWantedSpeed, this, ros::TransportHints().tcpNoDelay());
            currentSpeedSubscriber = create_subscription<>("carstate/speed", 1, &EmergencyStopNodelet::onCurrentSpeed, this, ros::TransportHints().tcpNoDelay());

            config_server_ = boost::make_shared<dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig> >(pnh);
            dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig>::CallbackType f;
            f = boost::bind(&EmergencyStopNodelet::callbackReconfigure, this, _1, _2);
            config_server_->setCallback(f);
        }

    private:
        /** Callback for messages of some type.
         **
         ** @param msg
         */
        void onScan(sensor_msgs::msg::LaserScanConstPtr const &msg) {
            emergencyStop->checkEmergencyStop(msg);
            speedPublisher.publish(emergencyStop->getSafeSpeed());
        }

        void onCurrentSpeed(autominy_msgs::msg::SpeedConstPtr const &msg) {
            emergencyStop->setCurrentSpeed(msg);
        }

        void onWantedSpeed(autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr const &msg) {
            emergencyStop->setWantedSpeed(msg);
        }

        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(emergency_stop::EmergencyStopConfig &config, uint32_t level) {
            emergencyStop->setConfig(config);
        }

        /// subscriber
        rclcpp::Subscription<>::SharedPtr scanSubscriber;
        rclcpp::Subscription<>::SharedPtr currentSpeedSubscriber;
        rclcpp::Subscription<>::SharedPtr wantedSpeedSubscriber;

        /// publisher
        rclcpp::Publisher<>::SharedPtr speedPublisher;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<emergency_stop::EmergencyStopConfig> > config_server_;

        /// pointer to the functionality class
        EmergencyStopPtr emergencyStop;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(emergency_stop::EmergencyStopNodelet, nodelet::Nodelet);
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
