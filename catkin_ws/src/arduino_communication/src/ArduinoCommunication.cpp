#include <memory>

#include "COBS.h"
#include "ArduinoCommunication.h"

namespace arduino_communication {

ArduinoCommunication::ArduinoCommunication(ros::NodeHandle &nh) {
    device = nh.param<std::string>("device", "/dev/ttyUSB1");
    baudrate = nh.param("baud", 115200);

    steeringAnglePublisher = nh.advertise<autominy_msgs::SteeringFeedback>("steering_angle", 2);
    voltagePublisher = nh.advertise<autominy_msgs::Voltage>("voltage", 2);
    ticksPublisher = nh.advertise<autominy_msgs::Tick>("ticks", 2);
    imuPublisher = nh.advertise<sensor_msgs::Imu>("imu", 2);
    imuTemperaturePublisher = nh.advertise<sensor_msgs::Temperature>("imu/temperature", 2);

    heartbeatTimer = nh.createTimer(ros::Duration(0.01), &ArduinoCommunication::onHeartbeat, this);
    speedSubscriber = nh.subscribe("speed", 2, &ArduinoCommunication::onSpeedCommand, this, ros::TransportHints().tcpNoDelay());
    steeringSubscriber = nh.subscribe("steering", 2, &ArduinoCommunication::onSteeringCommand, this, ros::TransportHints().tcpNoDelay());
    ledSubscriber = nh.subscribe("led", 2, &ArduinoCommunication::onLedCommand, this, ros::TransportHints().tcpNoDelay());
    imuCalibrationService = nh.advertiseService("calibrate_imu", &ArduinoCommunication::calibrateIMU, this);
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

size_t ArduinoCommunication::onSend(uint8_t *message, size_t length) {
    try {
        if (serial && serial->isOpen()) {
            return serial->write(message, length);
        } else {
            ROS_ERROR_THROTTLE(1, "Could not send to the Arduino!");
        }
    } catch(const std::exception& exception) {
        ROS_ERROR_THROTTLE(1, "Could not send to the Arduino! %s", exception.what());
    }

    return 0;
}

void ArduinoCommunication::spin() {

    uint8_t receiveBuffer[512];
    size_t receiveBufferIndex = 0;

    bool connected = false;

    ros::Rate r(100);
    while (ros::ok()) {
        try {

            if (!connected) {
                serial = std::make_unique<serial::Serial>(device, baudrate);
                connected = serial->isOpen();
            }

            while (connected && serial->available() > 0) {
                uint8_t recv;
                size_t bytes = serial->read(&recv, 1);
                if (bytes > 0) {
                    if (recv == 0) {
                        uint8_t decodeBuffer[receiveBufferIndex];

                        size_t numDecoded = COBS::decode(receiveBuffer,
                                                         receiveBufferIndex,
                                                         decodeBuffer);

                        onReceive(decodeBuffer, numDecoded);

                    } else {
                        if (receiveBufferIndex + 1 < 512) {
                            receiveBuffer[receiveBufferIndex++] = recv;
                        } else {
                            ROS_ERROR("Buffer overflow");
                        }
                    }
                }
            }
        } catch (const std::exception& exception) {
            connected = false;
            ROS_ERROR_THROTTLE(1, "Could not connect to arduino %s", exception.what());
        }

        ros::spinOnce();
        r.sleep();
    }
}

void ArduinoCommunication::onSteeringAngle(uint8_t *message) {
    uint16_t angle;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[2]),
              reinterpret_cast<char *>(&angle));

    autominy_msgs::SteeringFeedback msg;
    msg.value = angle;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    steeringAnglePublisher.publish(msg);

}
void ArduinoCommunication::onVoltage(uint8_t *message) {
    float voltage;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[4]),
              reinterpret_cast<char *>(&voltage));

    autominy_msgs::Voltage msg;
    msg.value = voltage;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    voltagePublisher.publish(msg);
}
void ArduinoCommunication::onIMU(uint8_t *message) {
    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = "imu";
    imuMsg.header.stamp = ros::Time::now();


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
    sensor_msgs::Temperature temperatureMsg;
    temperatureMsg.header.stamp = ros::Time::now();
    temperatureMsg.header.frame_id = "imu";
    temperatureMsg.temperature = (temperature / 340.0 ) + 36.53;
    imuTemperaturePublisher.publish(temperatureMsg);

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

    imuPublisher.publish(imuMsg);
}

void ArduinoCommunication::onWarn(uint8_t *message) {
    ROS_WARN("%s", message);
}

void ArduinoCommunication::onError(uint8_t *message) {
    ROS_ERROR("%s", message);
}

void ArduinoCommunication::onInfo(uint8_t *message) {
    ROS_INFO("%s", message);
}

void ArduinoCommunication::onDebug(uint8_t *message) {
    ROS_DEBUG("%s", message);
}

void ArduinoCommunication::onTicks(uint8_t *message) {
    autominy_msgs::Tick msg;
    uint8_t ticks;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[1]),
              reinterpret_cast<char *>(&ticks));

    msg.value = ticks;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_link";
    ticksPublisher.publish(msg);
}

void ArduinoCommunication::onSteeringCommand(autominy_msgs::SteeringCommandConstPtr const &steering) {
    uint8_t size = sizeof(MessageType) + sizeof(int16_t);
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::STEERING_CMD;
    memcpy(&message[1], &steering->value, sizeof(int16_t));
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    onSend(output, cobs);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

void ArduinoCommunication::onLedCommand(std_msgs::StringConstPtr const &led) {
    uint8_t size = sizeof(MessageType) + led->data.length() + 1;
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::LED_CMD;
    memcpy(&message[1], led->data.c_str(), led->data.length() + 1);
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    onSend(output, cobs);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }

}

void ArduinoCommunication::onHeartbeat(ros::TimerEvent const &event) {
    uint8_t size = sizeof(MessageType) + 1;
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::HEARTBEAT;
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    onSend(output, cobs);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

void ArduinoCommunication::onSpeedCommand(autominy_msgs::SpeedCommandConstPtr const &speed) {
    uint8_t size = sizeof(MessageType) + sizeof(int16_t);
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::SPEED_CMD;
    memcpy(&message[1], &speed->value, sizeof(int16_t));
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

bool ArduinoCommunication::calibrateIMU(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& resp) {
    uint8_t size = sizeof(MessageType);
    uint8_t message[size];

    message[0] = (uint8_t) MessageType::IMU_CALIBRATION;
    auto outputSize = COBS::getEncodedBufferSize(size);
    uint8_t output[outputSize];
    auto cobs = COBS::encode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }

    return true;
}

}
