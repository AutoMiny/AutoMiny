#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SpeedCommand.h>
#include <autominy_msgs/SteeringAngle.h>
#include <autominy_msgs/SteeringCommand.h>
#include <autominy_msgs/SteeringFeedback.h>
#include <autominy_msgs/Tick.h>
#include <autominy_msgs/Voltage.h>
#include <geometry_msgs/Twist.h>

#include <serial/serial.h>
#include <std_srvs/Empty.h>

#include "MessageType.h"

namespace arduino_communication {

class ArduinoCommunication {
 public:
    explicit ArduinoCommunication(ros::NodeHandle &nh);

    void spin();
 private:
    void onSpeedCommand(autominy_msgs::SpeedCommandConstPtr const &speed);
    void onSteeringCommand(autominy_msgs::SteeringCommandConstPtr const &steering);
    void onLedCommand(std_msgs::StringConstPtr const &led);
    void onHeartbeat(ros::TimerEvent const &event);

    void onReceive(uint8_t *message, size_t length);
    size_t onSend(uint8_t *message, size_t length);

    void onVoltage(uint8_t *message);
    void onSteeringAngle(uint8_t *message);
    void onIMU(uint8_t *message);
    void onError(uint8_t *message);
    void onWarn(uint8_t *message);
    void onInfo(uint8_t *message);
    void onDebug(uint8_t *message);
    void onTicks(uint8_t *message);
    bool calibrateIMU(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp);

    size_t cobsDecode(const uint8_t *input, size_t length, uint8_t *output);
    size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output);

    ros::Publisher ticksPublisher;
    ros::Publisher steeringAnglePublisher;
    ros::Publisher voltagePublisher;
    ros::Publisher imuPublisher;
    ros::Publisher imuTemperaturePublisher;
    ros::Subscriber speedSubscriber;
    ros::Subscriber steeringSubscriber;
    ros::Subscriber ledSubscriber;
    ros::Timer heartbeatTimer;
    ros::ServiceServer imuCalibrationService;

    std::string device;
    uint32_t baudrate;
    std::unique_ptr<serial::Serial> serial;
};

}