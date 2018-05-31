#pragma once

#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "SpeedCalibrationData.h"

namespace fub_speed_calibration {

class SpeedCalibration {
 public:
    explicit SpeedCalibration(ros::NodeHandle& nh);

 private:
    void onTwist(geometry_msgs::TwistPtr const &twist);
    void onIncreaseSpeed(ros::TimerEvent const &event);
    void onStartMeasurement(ros::TimerEvent const &event);
    void onStopMeasurement(ros::TimerEvent const &event);

    ros::Publisher speedPublisher;
    ros::Subscriber twistSubscriber;
    ros::Timer startMeasurementTimer;
    ros::Timer stopMeasurementTimer;
    ros::Timer increaseSpeedTimer;

    std::vector<geometry_msgs::TwistPtr> measurements;
    std::vector<fub_speed_calibration::SpeedCalibrationData> calibrationData;
    std_msgs::Int16 speedMsg;

    std::string fileName;
};

}