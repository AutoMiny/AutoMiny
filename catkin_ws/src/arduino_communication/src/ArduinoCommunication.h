#pragma once

#include <ros/ros.h>

#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <serial/serial.h>

#include "MessageType.h"

namespace arduino_communication {

class ArduinoCommunication {
 public:
    explicit ArduinoCommunication(ros::NodeHandle &nh);

    void spin();
 private:
    void onSpeedCommand(std_msgs::Int16 const &speed);
    void onSteeringCommand(std_msgs::UInt8 const &steering);
    void onLedCommand(std_msgs::String const &led);

    void onReceive(uint8_t *message, size_t length);
    size_t onSend(uint8_t *message, size_t length);

    void onVoltage(uint8_t *message);
    void onSteeringAngle(uint8_t *message);
    void onIMU(uint8_t *message);
    void onSpeed(uint8_t *message);
    void onError(uint8_t *message);
    void onWarn(uint8_t *message);
    void onInfo(uint8_t *message);
    void onDebug(uint8_t *message);
    void onTicks(uint8_t *message);

    size_t cobsDecode(const uint8_t *input, size_t length, uint8_t *output);
    size_t cobsEncode(const uint8_t *input, size_t length, uint8_t *output);

    ros::Publisher twistPublisher;
    ros::Publisher ticksPublisher;
    ros::Publisher steeringAnglePublisher;
    ros::Publisher voltagePublisher;
    ros::Publisher imuPublisher;
    ros::Subscriber speedSubscriber;
    ros::Subscriber steeringSubscriber;
    ros::Subscriber ledSubscriber;

    std::string device;
    uint32_t baudrate;
    std::unique_ptr<serial::Serial> serial;
};

}