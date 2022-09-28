#pragma once

#include <nodelet/nodelet.h>
#include "rclcpp/rclcpp.hpp"
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include "autominy_msgs/msg/tick.hpp"
#include "autominy_msgs/msg/steering_feedback.hpp"
#include "autominy_msgs/msg/steering_angle.hpp"
#include <autominy_msgs/NormalizedSteeringCommand.h>
#include <autominy_msgs/NormalizedSpeedCommand.h>
#include "autominy_msgs/msg/speed.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include <autominy_msgs/SteeringCommand.h>
#include "autominy_msgs/msg/steering_pwm_command.hpp"
#include <autominy_msgs/SpeedCommand.h>
#include <hardware_calibration/HardwareCalibrationConfig.h>
#include <boost/algorithm/clamp.hpp>
#include <boost/circular_buffer.hpp>
#include <numeric>

namespace hardware_calibration {
    enum class Direction : int8_t {
        FORWARD = 1,
        BACKWARD = -1
    };

    class HardwareCalibrationNodelet : public nodelet::Nodelet {

    public:
        HardwareCalibrationNodelet();
        void onInit() override;
        void onTicks(const autominy_msgs::msg::TickConstPtr& msg);
        void onSteeringFeedback(const autominy_msgs::msg::SteeringFeedbackConstPtr& msg);
        void onWantedSpeed(const autominy_msgs::msg::NormalizedSpeedCommandConstPtr& msg);
        void onWantedSteering(const autominy_msgs::msg::NormalizedSteeringCommandConstPtr& msg);
        void onSpeedCommand(const autominy_msgs::msg::SpeedCommandConstPtr& msg);
        void onSteeringCommand(const autominy_msgs::msg::SteeringCommandConstPtr& msg);
        void onReconfigure(HardwareCalibrationConfig &config, uint32_t level);
    private:
        double mapRange(double a1, double a2, double b1, double b2, double s);

        /// subscriber
        rclcpp::Subscription<>::SharedPtr steeringFeedbackSubscriber;
        rclcpp::Subscription<>::SharedPtr speedSubscriber;
        rclcpp::Subscription<>::SharedPtr steeringSubscriber;
        rclcpp::Subscription<>::SharedPtr wantedSpeedSubscriber;
        rclcpp::Subscription<>::SharedPtr wantedSteeringSubscriber;
        rclcpp::Subscription<>::SharedPtr ticksSubscriber;

        /// Publisher
        rclcpp::Publisher<>::SharedPtr steeringPublisher;
        rclcpp::Publisher<>::SharedPtr speedPublisher;
        rclcpp::Publisher<>::SharedPtr steeringAnglePublisher;
        rclcpp::Publisher<>::SharedPtr speedMPSPublisher;
        rclcpp::Publisher<>::SharedPtr calibratedSpeedPublisher;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<HardwareCalibrationConfig>> configServer;
        HardwareCalibrationConfig config;

        boost::circular_buffer<int16_t> steeringFeedbackBuffer;
        boost::circular_buffer<autominy_msgs::msg::TickConstPtr> ticksBuffer;
        Direction direction;
        Direction wantedDirection;
        double currentSpeed;
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hardware_calibration::HardwareCalibrationNodelet, nodelet::Nodelet);
