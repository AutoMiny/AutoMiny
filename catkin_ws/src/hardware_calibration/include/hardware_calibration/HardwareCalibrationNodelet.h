#pragma once

#include "rclcpp/rclcpp.hpp"
#include "autominy_msgs/msg/tick.hpp"
#include "autominy_msgs/msg/steering_feedback.hpp"
#include "autominy_msgs/msg/steering_angle.hpp"
#include "autominy_msgs/msg/normalized_steering_command.hpp"
#include "autominy_msgs/msg/normalized_speed_command.hpp"
#include "autominy_msgs/msg/speed.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include "autominy_msgs/msg/steering_command.hpp"
#include "autominy_msgs/msg/speed_command.hpp"
#include "autominy_msgs/msg/steering_pwm_command.hpp"
#include <boost/algorithm/clamp.hpp>
#include <boost/circular_buffer.hpp>
#include <numeric>

namespace hardware_calibration {

    struct HardwareCalibrationConfig {
        int minimum_steering_feedback = 192;
        int maximum_steering_feedback = 420;
        double minimum_steering_radians = 0.512;
        double maximum_steering_radians = -0.498;
        int minimum_steering_pwm = 950;
        int maximum_steering_pwm = 2150;
        int minimum_speed_pwm = -1000;
        int maximum_speed_pwm = 1000;
        double ticks_to_m = 0.0027;
        int number_of_ticks_filter = 20;
        int number_of_steering_msgs_filter = 10;
        double direction_change_max_speed = 0.3;
    };

    enum class Direction : int8_t {
        FORWARD = 1,
        BACKWARD = -1
    };

    class HardwareCalibrationNodelet : public rclcpp::Node {

    public:
        HardwareCalibrationNodelet(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
        void onTicks(const autominy_msgs::msg::Tick::ConstSharedPtr & msg);
        void onSteeringFeedback(const autominy_msgs::msg::SteeringFeedback::ConstSharedPtr& msg);
        void onWantedSpeed(const autominy_msgs::msg::NormalizedSpeedCommand::ConstSharedPtr& msg);
        void onWantedSteering(const autominy_msgs::msg::NormalizedSteeringCommand::ConstSharedPtr& msg);
        void onSpeedCommand(const autominy_msgs::msg::SpeedCommand::ConstSharedPtr& msg);
        void onSteeringCommand(const autominy_msgs::msg::SteeringCommand::ConstSharedPtr& msg);
    private:
        double mapRange(double a1, double a2, double b1, double b2, double s);

        /// subscriber
        rclcpp::Subscription<autominy_msgs::msg::SteeringFeedback>::SharedPtr steeringFeedbackSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::SpeedCommand>::SharedPtr speedSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::SteeringCommand>::SharedPtr steeringSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::NormalizedSpeedCommand>::SharedPtr wantedSpeedSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::NormalizedSteeringCommand>::SharedPtr wantedSteeringSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::Tick>::SharedPtr ticksSubscriber;

        /// Publisher
        rclcpp::Publisher<autominy_msgs::msg::SteeringPWMCommand>::SharedPtr steeringPublisher;
        rclcpp::Publisher<autominy_msgs::msg::SpeedPWMCommand>::SharedPtr speedPublisher;
        rclcpp::Publisher<autominy_msgs::msg::SteeringAngle>::SharedPtr steeringAnglePublisher;
        rclcpp::Publisher<autominy_msgs::msg::SpeedCommand>::SharedPtr speedMPSPublisher;
        rclcpp::Publisher<autominy_msgs::msg::Speed>::SharedPtr calibratedSpeedPublisher;

        /// pointer to dynamic reconfigure service
        HardwareCalibrationConfig config;

        boost::circular_buffer<int16_t> steeringFeedbackBuffer;
        boost::circular_buffer<autominy_msgs::msg::Tick::ConstSharedPtr> ticksBuffer;
        Direction direction;
        Direction wantedDirection;
        double currentSpeed;
    };
}

