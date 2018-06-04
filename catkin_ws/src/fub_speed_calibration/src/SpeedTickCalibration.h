#pragma once

#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Twist.h>

namespace fub_speed_calibration {

class SpeedTickCalibration {
 public:
    explicit SpeedTickCalibration(ros::NodeHandle& nh);

 private:
    void onTick(std_msgs::UInt16Ptr const &tick);
    void onStartMeasurement(ros::TimerEvent const &event);
    void onStopMeasurement(ros::TimerEvent const &event);
    void onBrake(ros::TimerEvent const &event);

    ros::Subscriber tickSubscriber;
    ros::Publisher speedPublisher;
    ros::Timer startMeasurementTimer;
    ros::Timer stopMeasurementTimer;
    ros::Timer breakTimer;

    double calibrationTime;
    double brakeTime;
    unsigned int ticks = 0;
    std_msgs::Int16 speedMsg;
};

}