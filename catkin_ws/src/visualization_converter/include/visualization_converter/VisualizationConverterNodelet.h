#pragma once

#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"
#include "autominy_msgs/msg/steering_angle.hpp"
#include "autominy_msgs/msg/speed.hpp"
#include <autominy_msgs/SteeringCommand.h>
#include <autominy_msgs/SpeedCommand.h>
#include <autominy_msgs/Plot.h>
#include "autominy_msgs/msg/voltage.hpp"

namespace visualization_converter {

    class VisualizationConverterNodelet : public nodelet::Nodelet {

    public:
        void onInit() override;
        void onSteeringAngle(const autominy_msgs::msg::SteeringAngleConstPtr& msg);
        void onSpeed(const autominy_msgs::msg::SpeedConstPtr& msg);
        void onWantedSpeed(const autominy_msgs::msg::SpeedCommandConstPtr& msg);
        void onWantedSteering(const autominy_msgs::msg::SteeringCommandConstPtr& msg);
        void onVoltage(const autominy_msgs::msg::VoltageConstPtr& msg);

    private:
        /// subscriber
        rclcpp::Subscription<>::SharedPtr steeringAngleSubscriber;
        rclcpp::Subscription<>::SharedPtr speedSubscriber;
        rclcpp::Subscription<>::SharedPtr wantedSpeedSubscriber;
        rclcpp::Subscription<>::SharedPtr wantedSteeringSubscriber;
        rclcpp::Subscription<>::SharedPtr voltageSubscriber;

        /// Publisher
        rclcpp::Publisher<>::SharedPtr steeringAnglePublisher;
        rclcpp::Publisher<>::SharedPtr speedPublisher;
        rclcpp::Publisher<>::SharedPtr wantedSpeedPublisher;
        rclcpp::Publisher<>::SharedPtr wantedSteeringPublisher;
        rclcpp::Publisher<>::SharedPtr voltagePublisher;
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualization_converter::VisualizationConverterNodelet, nodelet::Nodelet);
