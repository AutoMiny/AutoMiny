#pragma once

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "autominy_msgs/msg/speed.hpp"
#include "autominy_msgs/msg/speed_pwm_command.hpp"
#include "autominy_msgs/msg/steering_angle.hpp"
#include "autominy_msgs/msg/steering_pwm_command.hpp"
#include "autominy_msgs/msg/steering_feedback.hpp"
#include "autominy_msgs/msg/tick.hpp"
#include "autominy_msgs/msg/voltage.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "serial/serial.h"
#include "std_srvs/srv/empty.hpp"

#include "MessageType.h"

namespace arduino_communication {

class ArduinoCommunication : public rclcpp::Node {
 public:
    explicit ArduinoCommunication(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    void spin();
 private:
    void onSpeedCommand(const autominy_msgs::msg::SpeedPWMCommand::SharedPtr speed);
    void onSteeringCommand(const autominy_msgs::msg::SteeringPWMCommand::SharedPtr steering);
    void onLedCommand(const std_msgs::msg::String::SharedPtr led);
    void onHeartbeat();

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
    bool calibrateIMU(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr resp);

    rclcpp::Publisher<autominy_msgs::msg::Tick>::SharedPtr ticksPublisher;
    rclcpp::Publisher<autominy_msgs::msg::SteeringFeedback>::SharedPtr steeringAnglePublisher;
    rclcpp::Publisher<autominy_msgs::msg::Voltage>::SharedPtr voltagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr imuTemperaturePublisher;
    rclcpp::Subscription<autominy_msgs::msg::SpeedPWMCommand>::SharedPtr speedSubscriber;
    rclcpp::Subscription<autominy_msgs::msg::SteeringPWMCommand>::SharedPtr steeringSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ledSubscriber;
    rclcpp::TimerBase::SharedPtr heartbeatTimer;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr imuCalibrationService;

    std::string device;
    uint32_t baudrate;
    std::unique_ptr<serial::Serial> serial;
};

}