#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"

#include <dynamic_reconfigure/server.h>
#include <odometry/OdometryFwd.h>
#include <odometry/OdometryConfig.h>
#include <odometry/Odometry.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include "autominy_msgs/msg/speed.hpp"
#include <nav_msgs/Odometry.h>

namespace odometry {

/** Odometry nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class OdometryNodelet : public nodelet::Nodelet {
    public:
        /** Constructor.
         */
        OdometryNodelet() = default;

        /** Destructor.
         */
        ~OdometryNodelet() override {}

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        virtual void onInit() override {
            ros::NodeHandle nh = getNodeHandle();
            ros::NodeHandle pnh = getPrivateNodeHandle();

            odometry = std::make_shared<Odometry>();

            odometryPublisher = pcreate_publisher<nav_msgs::Odometry>("odom", 1);
            speedSubscriber = create_subscription<>("speed", 1, &OdometryNodelet::onSpeed, this, ros::TransportHints().tcpNoDelay());
            steeringSubscriber = create_subscription<>("steering", 1, &OdometryNodelet::onSteering, this, ros::TransportHints().tcpNoDelay());

            config_server_ = boost::make_shared<dynamic_reconfigure::Server<odometry::OdometryConfig> >(pnh);
            dynamic_reconfigure::Server<odometry::OdometryConfig>::CallbackType f;
            f = boost::bind(&OdometryNodelet::callbackReconfigure, this, _1, _2);
            config_server_->setCallback(f);

            timer = prclcpp::create_timer(rclcpp::Duration::from_seconds(0.01), &OdometryNodelet::onOdometry, this);
        }

    private:

        void onOdometry(const ros::TimerEvent& event) {
            auto msg = odometry->step(event);
            odometryPublisher.publish(msg);
        }

        void onSpeed(autominy_msgs::msg::Speed::ConstSharedPtr const &msg) {
            odometry->setSpeed(msg);
        }

        void onSteering(autominy_msgs::msg::SteeringAngleConstPtr const &msg) {
            odometry->setSteering(msg);
        }

        /** Callback for dynamic_reconfigure.
         **
         ** @param msg
         */
        void callbackReconfigure(odometry::OdometryConfig &config, uint32_t level) {
            odometry->setConfig(config);
        }

        /// subscriber
        rclcpp::Subscription<>::SharedPtr speedSubscriber;
        rclcpp::Subscription<>::SharedPtr steeringSubscriber;

        /// publisher
        rclcpp::Publisher<>::SharedPtr odometryPublisher;

        ros::Timer timer;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<odometry::OdometryConfig> > config_server_;

        /// pointer to the functionality class
        OdometryPtr odometry;
    };
}

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(odometry::OdometryNodelet, nodelet::Nodelet);
// nodelet_plugins.xml refers to the parameters as "type" and "base_class_type"
