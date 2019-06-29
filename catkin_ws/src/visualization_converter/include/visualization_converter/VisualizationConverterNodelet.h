#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <autominy_msgs/SteeringAngle.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringCommand.h>
#include <autominy_msgs/SpeedCommand.h>
#include <autominy_msgs/Plot.h>
#include <autominy_msgs/Voltage.h>

namespace visualization_converter {

    class VisualizationConverterNodelet : public nodelet::Nodelet {

    public:
        void onInit() override;
        void onSteeringAngle(const autominy_msgs::SteeringAngleConstPtr& msg);
        void onSpeed(const autominy_msgs::SpeedConstPtr& msg);
        void onWantedSpeed(const autominy_msgs::SpeedCommandConstPtr& msg);
        void onWantedSteering(const autominy_msgs::SteeringCommandConstPtr& msg);
        void onVoltage(const autominy_msgs::VoltageConstPtr& msg);

    private:
        /// subscriber
        ros::Subscriber steeringAngleSubscriber;
        ros::Subscriber speedSubscriber;
        ros::Subscriber wantedSpeedSubscriber;
        ros::Subscriber wantedSteeringSubscriber;
        ros::Subscriber voltageSubscriber;

        /// Publisher
        ros::Publisher steeringAnglePublisher;
        ros::Publisher speedPublisher;
        ros::Publisher wantedSpeedPublisher;
        ros::Publisher wantedSteeringPublisher;
        ros::Publisher voltagePublisher;
    };
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(visualization_converter::VisualizationConverterNodelet, nodelet::Nodelet);
