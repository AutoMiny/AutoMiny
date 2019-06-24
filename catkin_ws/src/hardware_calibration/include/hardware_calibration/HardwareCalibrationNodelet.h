#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <autominy_msgs/Tick.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/SteeringAngle.h>
#include <autominy_msgs/NormalizedSteeringCommand.h>
#include <autominy_msgs/NormalizedSpeedCommand.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SpeedPWMCommand.h>
#include <autominy_msgs/SteeringCommand.h>
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
        void onInit() override;
        void onTicks(const autominy_msgs::TickConstPtr& msg);
        void onSteeringFeedback(const autominy_msgs::SteeringFeedbackConstPtr& msg);
        void onWantedSpeed(const autominy_msgs::NormalizedSpeedCommandConstPtr& msg);
        void onWantedSteering(const autominy_msgs::NormalizedSteeringCommandConstPtr& msg);
        void onSpeedCommand(const autominy_msgs::SpeedCommandConstPtr& msg);
        void onReconfigure(HardwareCalibrationConfig &config, uint32_t level);
    private:
        double mapRange(double a1, double a2, double b1, double b2, double s);

        /// subscriber
        ros::Subscriber steeringFeedbackSubscriber;
        ros::Subscriber speedSubscriber;
        ros::Subscriber wantedSpeedSubscriber;
        ros::Subscriber wantedSteeringSubscriber;
        ros::Subscriber ticksSubscriber;

        /// Publisher
        ros::Publisher steeringPublisher;
        ros::Publisher speedPublisher;
        ros::Publisher steeringAnglePublisher;
        ros::Publisher calibratedSpeedPublisher;

        /// pointer to dynamic reconfigure service
        boost::shared_ptr<dynamic_reconfigure::Server<HardwareCalibrationConfig>> configServer;
        HardwareCalibrationConfig config;

        boost::circular_buffer<int16_t> steeringFeedbackBuffer;
        boost::circular_buffer<autominy_msgs::TickConstPtr> ticksBuffer;
        Direction direction = Direction::FORWARD;
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hardware_calibration::HardwareCalibrationNodelet, nodelet::Nodelet);
