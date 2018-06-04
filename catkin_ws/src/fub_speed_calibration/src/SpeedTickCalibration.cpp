#include "SpeedTickCalibration.h"
#include <fstream>

namespace fub_speed_calibration {

SpeedTickCalibration::SpeedTickCalibration(ros::NodeHandle& nh) {
    tickSubscriber = nh.subscribe("/ticks", 1, &SpeedTickCalibration::onTick, this);
    speedPublisher = nh.advertise<std_msgs::Int16>("/manual_control/speed", 1);

    startMeasurementTimer = nh.createTimer(ros::Duration(1), &SpeedTickCalibration::onStartMeasurement, this, true, true);
    breakTimer = nh.createTimer(ros::Duration(10), &SpeedTickCalibration::onBrake, this, true, false);
    stopMeasurementTimer = nh.createTimer(ros::Duration(10), &SpeedTickCalibration::onStopMeasurement, this, true, false);

    calibrationTime = nh.param<double>("calibration_time", 10.0);
    brakeTime = nh.param<double>("break_time", 5.0);
    speedMsg.data = nh.param("speed", 200);
}

void SpeedTickCalibration::onTick(std_msgs::UInt16Ptr const &tick) {
    ticks += tick->data;
}

void SpeedTickCalibration::onBrake(ros::TimerEvent const &event) {
    stopMeasurementTimer.stop();
    stopMeasurementTimer.setPeriod(ros::Duration(brakeTime));
    stopMeasurementTimer.start();

    ROS_INFO("Braking");
}

void SpeedTickCalibration::onStartMeasurement(ros::TimerEvent const &event) {
    breakTimer.stop();
    breakTimer.setPeriod(ros::Duration(calibrationTime));
    breakTimer.start();

    ROS_INFO("Starting calibration");
}

void SpeedTickCalibration::onStopMeasurement(ros::TimerEvent const &event) {
    speedMsg.data = 0;
    speedPublisher.publish(speedMsg);
    ROS_INFO("Total number of ticks: %d", ticks);

    ros::shutdown();
}

}
