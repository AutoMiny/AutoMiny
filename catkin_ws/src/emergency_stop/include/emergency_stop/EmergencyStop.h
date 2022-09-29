#pragma once

#include "rclcpp/rclcpp.hpp"

#include <emergency_stop/EmergencyStopFwd.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include "autominy_msgs/msg/speed.hpp"

namespace emergency_stop {

    struct EmergencyStopConfig {
        double angle_front = 0.7;
        double angle_back = 0.7;
        double break_distance = 0.45;
        bool break_distance_based_on_speed = false;
        double reverse_minimum_distance = 0.28;
        double forward_minimum_distance = 0.07;
        double negative_acceleration = 4.0;
    };

    /** EmergencyStop nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class EmergencyStopNodelet : public rclcpp::Node {
    public:
        /** Destructor.
         */
        ~EmergencyStopNodelet() override;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        EmergencyStopNodelet(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    private:
        /** Callback for messages of some type.
         **
         ** @param msg
         */
        void onScan(sensor_msgs::msg::LaserScan::ConstSharedPtr const &scan);

        void onCurrentSpeed(autominy_msgs::msg::Speed::ConstSharedPtr const &msg);

        void onWantedSpeed(autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr const &msg);

        autominy_msgs::msg::SpeedPWMCommand getSafeSpeed();

        void setCurrentSpeed(const autominy_msgs::msg::Speed::ConstSharedPtr &speed);

        void setWantedSpeed(const autominy_msgs::msg::SpeedPWMCommand::ConstSharedPtr &speed);

        rcl_interfaces::msg::SetParametersResult onConfig(const std::vector<rclcpp::Parameter>& params);


        /// subscriber
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::Speed>::SharedPtr currentSpeedSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::SpeedPWMCommand>::SharedPtr wantedSpeedSubscriber;

        /// publisher
        rclcpp::Publisher<autominy_msgs::msg::SpeedPWMCommand>::SharedPtr speedPublisher;

        emergency_stop::EmergencyStopConfig config;
        double currentSpeed = 0.0;
        int16_t wantedSpeed = 0;
        bool emergencyStop = true;
        OnSetParametersCallbackHandle::SharedPtr cb;
    };
}