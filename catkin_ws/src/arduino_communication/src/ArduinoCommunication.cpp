#include <memory>

#include "COBS.h"
#include "ArduinoCommunication.h"

namespace arduino_communication {

ArduinoCommunication::ArduinoCommunication(const rclcpp::NodeOptions& opts) : rclcpp::Node("arduino_communication", opts) {
    device = declare_parameter<std::string>("device", "/dev/ttyUSB0");
    baudrate = declare_parameter<int>("baud", 115200);

    steeringAnglePublisher = create_publisher<autominy_msgs::msg::SteeringFeedback>("steering_angle", 1);
    voltagePublisher = create_publisher<autominy_msgs::msg::Voltage>("voltage", 1);
    ticksPublisher = create_publisher<autominy_msgs::msg::Tick>("ticks", 1);
    imuPublisher = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    imuTemperaturePublisher = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 1);

    heartbeatTimer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01), std::bind(&ArduinoCommunication::onHeartbeat, this));
    speedSubscriber = create_subscription<autominy_msgs::msg::SpeedPWMCommand>("speed", 1, std::bind(&ArduinoCommunication::onSpeedCommand, this, std::placeholders::_1));
    steeringSubscriber = create_subscription<autominy_msgs::msg::SteeringPWMCommand>("steering", 1, std::bind(&ArduinoCommunication::onSteeringCommand, this, std::placeholders::_1));
    ledSubscriber = create_subscription<std_msgs::msg::String>("led", 1, std::bind(&ArduinoCommunication::onLedCommand, this, std::placeholders::_1));
    imuCalibrationService = create_service<std_srvs::srv::Empty>("calibrate_imu", std::bind(&ArduinoCommunication::calibrateIMU, this, std::placeholders::_1, std::placeholders::_2));
}

void ArduinoCommunication::onReceive(uint8_t *message, size_t length) {
    if (length > 0) {
        auto type = static_cast<MessageType>(message[0]);
        switch (type) {
            case MessageType::DEBUG:onDebug(&message[1]);
                break;
            case MessageType::INFO:onInfo(&message[1]);
                break;

            case MessageType::WARN:onWarn(&message[1]);
                break;

            case MessageType::ERROR:onError(&message[1]);
                break;

            case MessageType::SPEED_CMD:break;
            case MessageType::STEERING_CMD:break;
            case MessageType::LED_CMD:break;
            case MessageType::HEARTBEAT:break;
            case MessageType::IMU_CALIBRATION:break;
            case MessageType::STEERING_ANGLE:onSteeringAngle(&message[1]);
                break;
            case MessageType::TICKS:onTicks(&message[1]);
                break;
            case MessageType::IMU:onIMU(&message[1]);
                break;
            case MessageType::VOLTAGE:onVoltage(&message[1]);
                break;
        }
    }
}

size_t ArduinoCommunication::onSend(uint8_t* message, size_t length) {
    try {
        if (serial && serial->isOpen()) {
            return serial->write(message, length);
        } else {
            auto clk = *get_clock();
            RCLCPP_ERROR_THROTTLE(get_logger(), clk, 1000, "Could not send to the Arduino!");
        }
    } catch(const std::exception& exception) {
        auto clk = *get_clock();
        RCLCPP_ERROR_THROTTLE(get_logger(), clk, 1000, "Could not send to the Arduino! %s", exception.what());
    }

    return 0;
}

void ArduinoCommunication::spin() {

    uint8_t receiveBuffer[512];
    size_t receiveBufferIndex = 0;

    bool connected = false;

    rclcpp::Rate r(200);
    while (rclcpp::ok()) {
        try {

            if (!connected) {
                serial = std::make_unique<serial::Serial>(device, baudrate);
                connected = serial->isOpen();
            }

            while (connected && serial->available() > 0) {
                uint8_t buf[128];
                size_t bytes = serial->read(buf, 128);
                for (int i = 0; i < bytes; i++) {
                    if (buf[i] == 0) {
                        uint8_t decodeBuffer[receiveBufferIndex];

                        size_t numDecoded = COBS::decode(receiveBuffer,
                                                         receiveBufferIndex,
                                                         decodeBuffer);

                        onReceive(decodeBuffer, numDecoded);
                        receiveBufferIndex = 0;
                    } else {
                        if (receiveBufferIndex + 1 < 512) {
                            receiveBuffer[receiveBufferIndex++] = buf[i];
                        } else {
                            RCLCPP_ERROR(get_logger(), "Buffer overflow");
                        }
                    }
                }
            }
        } catch (const std::exception& exception) {
            connected = false;
            auto clk = *get_clock();
            RCLCPP_ERROR_THROTTLE(get_logger(), clk, 1000, "Could not connect to arduino %s", exception.what());
        }

        rclcpp::spin(this->shared_from_this());
        r.sleep();
    }
}

void ArduinoCommunication::onSteeringAngle(uint8_t *message) {
    uint16_t angle;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[2]),
              reinterpret_cast<char *>(&angle));

    autominy_msgs::msg::SteeringFeedback msg;
    msg.value = angle;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    steeringAnglePublisher->publish(msg);

}
void ArduinoCommunication::onVoltage(uint8_t *message) {
    float voltage;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[4]),
              reinterpret_cast<char *>(&voltage));

    autominy_msgs::msg::Voltage msg;
    msg.value = voltage;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    voltagePublisher->publish(msg);
}
void ArduinoCommunication::onIMU(uint8_t *message) {
    sensor_msgs::msg::Imu imuMsg;
    imuMsg.header.frame_id = "imu";
    imuMsg.header.stamp = now();


    int16_t w = (message[0] << 8) | message[1];
    int16_t x = (message[2] << 8) | message[3];
    int16_t y = (message[4] << 8) | message[5];
    int16_t z = (message[6] << 8) | message[7];

    double wf = w/16384.0;
    double xf = x/16384.0;
    double yf = y/16384.0;
    double zf = z/16384.0;

    int16_t gx = (message[8] << 8) | message[9];
    int16_t gy = (message[10] << 8) | message[11];
    int16_t gz = (message[12] << 8) | message[13];

    double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
    double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
    double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

    int16_t ax = (message[14] << 8) | message[15];
    int16_t ay = (message[16] << 8) | message[17];
    int16_t az = (message[18] << 8) | message[19];
    // calculate accelerations in m/s²
    double axf = ax * (8.0 / 65536.0) * 9.81;
    double ayf = ay * (8.0 / 65536.0) * 9.81;
    double azf = az * (8.0 / 65536.0) * 9.81;

    int16_t temperature = (message[20] << 8) | message[21];
    sensor_msgs::msg::Temperature temperatureMsg;
    temperatureMsg.header.stamp = now();
    temperatureMsg.header.frame_id = "imu";
    temperatureMsg.temperature = (temperature / 340.0 ) + 36.53;
    imuTemperaturePublisher->publish(temperatureMsg);

    // map from NED to ENU
    imuMsg.orientation.x = yf;
    imuMsg.orientation.y = -xf;
    imuMsg.orientation.z = zf;
    imuMsg.orientation.w = wf;

    imuMsg.angular_velocity.x = gyf;
    imuMsg.angular_velocity.y = -gxf;
    imuMsg.angular_velocity.z = gzf;

    imuMsg.linear_acceleration.x = ayf;
    imuMsg.linear_acceleration.y = -axf;
    imuMsg.linear_acceleration.z = azf;

    auto linearAccelerationStdDev = (400 / 1000000.0) * 9.807;
    auto angularVelocityStdDev = 0.05 * (M_PI / 180.0);
    auto pitchRollStdDev =  1.0 * (M_PI / 180.0);
    auto yawStdDev =  5.0 * (M_PI / 180.0);

    imuMsg.linear_acceleration_covariance[0] = linearAccelerationStdDev * linearAccelerationStdDev;
    imuMsg.linear_acceleration_covariance[4] = linearAccelerationStdDev * linearAccelerationStdDev;
    imuMsg.linear_acceleration_covariance[8] = linearAccelerationStdDev * linearAccelerationStdDev;

    imuMsg.angular_velocity_covariance[0] = angularVelocityStdDev * angularVelocityStdDev;
    imuMsg.angular_velocity_covariance[4] = angularVelocityStdDev * angularVelocityStdDev;
    imuMsg.angular_velocity_covariance[8] = angularVelocityStdDev * angularVelocityStdDev;

    imuMsg.orientation_covariance[0] = pitchRollStdDev * pitchRollStdDev;
    imuMsg.orientation_covariance[4] = pitchRollStdDev * pitchRollStdDev;
    imuMsg.orientation_covariance[8] = yawStdDev * yawStdDev;

    imuPublisher->publish(imuMsg);
}

void ArduinoCommunication::onWarn(uint8_t *message) {
    RCLCPP_WARN(get_logger(), "%s", message);
}

void ArduinoCommunication::onError(uint8_t *message) {
    RCLCPP_ERROR(get_logger(), "%s", message);
}

void ArduinoCommunication::onInfo(uint8_t *message) {
    RCLCPP_INFO(get_logger(), "%s", message);
}

void ArduinoCommunication::onDebug(uint8_t *message) {
    RCLCPP_DEBUG(get_logger(), "%s", message);
}

void ArduinoCommunication::onTicks(uint8_t *message) {
    autominy_msgs::msg::Tick msg;
    uint8_t ticks;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[1]),
              reinterpret_cast<char *>(&ticks));

    msg.value = ticks;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    ticksPublisher->publish(msg);
}

void ArduinoCommunication::onSteeringCommand(const autominy_msgs::msg::SteeringPWMCommand::SharedPtr steering) {
    uint8_t size = sizeof(MessageType) + sizeof(int16_t);
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::STEERING_CMD;
    memcpy(&message[1], &steering->value, sizeof(int16_t));
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        RCLCPP_ERROR(get_logger(), "Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

void ArduinoCommunication::onLedCommand(const std_msgs::msg::String::SharedPtr led) {
    uint8_t size = sizeof(MessageType) + led->data.length() + 1;
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::LED_CMD;
    memcpy(&message[1], led->data.c_str(), led->data.length() + 1);
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        RCLCPP_ERROR(get_logger(), "Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }

}

void ArduinoCommunication::onHeartbeat() {
    uint8_t size = sizeof(MessageType);
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::HEARTBEAT;
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        RCLCPP_ERROR(get_logger(), "Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

void ArduinoCommunication::onSpeedCommand(const autominy_msgs::msg::SpeedPWMCommand::SharedPtr speed) {
    uint8_t size = sizeof(MessageType) + sizeof(int16_t);
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::SPEED_CMD;
    memcpy(&message[1], &speed->value, sizeof(int16_t));
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        RCLCPP_ERROR(get_logger(), "Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

bool ArduinoCommunication::calibrateIMU(const std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr resp) {
    uint8_t size = sizeof(MessageType);
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::IMU_CALIBRATION;
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        RCLCPP_ERROR(get_logger(), "Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }

    return true;
}

}
