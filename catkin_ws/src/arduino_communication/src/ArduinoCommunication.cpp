#include "ArduinoCommunication.h"

namespace arduino_communication {

ArduinoCommunication::ArduinoCommunication(ros::NodeHandle &nh) {
    device = nh.param<std::string>("device", "/dev/ttyUSB1");
    baudrate = nh.param("baud", 115200);

    speedSubscriber = nh.subscribe("/speed", 2, &ArduinoCommunication::onSpeedCommand, this);
    steeringSubscriber = nh.subscribe("/steering", 2, &ArduinoCommunication::onSteeringCommand, this);
    ledSubscriber = nh.subscribe("/led", 2, &ArduinoCommunication::onLedCommand, this);

    twistPublisher = nh.advertise<geometry_msgs::Twist>("/twist", 2);
    steeringAnglePublisher = nh.advertise<std_msgs::UInt16>("/steering_angle", 2);
    voltagePublisher = nh.advertise<std_msgs::Float32>("/voltage", 2);
    ticksPublisher = nh.advertise<std_msgs::UInt8>("/ticks", 2);
    yawPublisher = nh.advertise<std_msgs::Float32>("/yaw", 2);
    pitchPublisher = nh.advertise<std_msgs::Float32>("/pitch", 2);
    rollPublisher = nh.advertise<std_msgs::Float32>("/roll", 2);
}

size_t ArduinoCommunication::cobsEncode(const uint8_t *input, size_t length, uint8_t *output) {
    size_t write_idx = 1;
    size_t read_idx = 0;
    uint8_t code = 1;
    size_t code_idx = 0;

    while (read_idx < length) {
        if (input[read_idx] == 0) {
            output[code_idx] = code;
            code = 1;
            code_idx = write_idx++;
            read_idx++;
        } else {
            output[write_idx++] = input[read_idx++];
            code++;
            if (code == 0xFF) {
                output[code_idx] = code;
                code = 1;
                code_idx = write_idx++;
            }
        }
    }

    output[code_idx] = code;
    output[write_idx++] = 0;

    return write_idx;
}

size_t ArduinoCommunication::cobsDecode(const uint8_t *input, size_t length, uint8_t *output) {
    size_t write_idx = 0;
    size_t read_idx = 0;
    uint8_t i;
    uint8_t code;

    while (read_idx < length) {
        code = input[read_idx];

        if (read_idx + code > length && code != 1) {
            return 0;
        }

        read_idx++;

        for (i = 1; i < code; i++) {
            output[write_idx++] = input[read_idx++];
        }
        if (code != 0xFF && read_idx != length) {
            output[write_idx++] = '\0';
        }
    }

    return write_idx;
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
            case MessageType::STEERING_ANGLE:onSteeringAngle(&message[1]);
                break;
            case MessageType::TICKS:onTicks(&message[1]);
                break;
            case MessageType::SPEED:onSpeed(&message[1]);
                break;
            case MessageType::IMU:onIMU(&message[1]);
                break;
            case MessageType::VOLTAGE:onVoltage(&message[1]);
                break;
        }
    }
}

size_t ArduinoCommunication::onSend(uint8_t *message, size_t length) {
    if (serial && serial->isOpen()) {
        return serial->write(message, length);
    } else {
        ROS_ERROR_THROTTLE(1, "Could not send to the Arduino!");
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
                serial = std::unique_ptr<serial::Serial>(new serial::Serial(device, baudrate));
                connected = serial->isOpen();
            }

            while (connected && serial->available()) {
                uint8_t recv;
                size_t bytes = serial->read(&recv, 1);
                if (bytes > 0) {
                    receiveBuffer[receiveBufferIndex++] = recv;

                    if (recv == 0) {
                        uint8_t message[512];
                        size_t read = cobsDecode(receiveBuffer, receiveBufferIndex, message);

                        if (read == 0) {
                            ROS_WARN("Dropped corrupted package from arduino!");
                        } else {
                            onReceive(message, read);
                        }

                        receiveBufferIndex = 0;
                    }

                }
            }
        } catch (serial::IOException &exception) {
            connected = false;
            ROS_ERROR_THROTTLE(1, "Could not connect to arduino");
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

    std_msgs::UInt16 msg;
    msg.data = angle;
    steeringAnglePublisher.publish(msg);

}
void ArduinoCommunication::onVoltage(uint8_t *message) {
    float voltage;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[4]),
              reinterpret_cast<char *>(&voltage));

    std_msgs::Float32 msg;
    msg.data = voltage;
    voltagePublisher.publish(msg);
}
void ArduinoCommunication::onIMU(uint8_t *message) {
    float yaw, pitch, roll;
    std_msgs::Float32 yawMsg, pitchMsg, rollMsg;

    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[4]),
              reinterpret_cast<char *>(&yaw));

    std::copy(reinterpret_cast<const char *>(&message[4]),
              reinterpret_cast<const char *>(&message[8]),
              reinterpret_cast<char *>(&pitch));

    std::copy(reinterpret_cast<const char *>(&message[8]),
              reinterpret_cast<const char *>(&message[12]),
              reinterpret_cast<char *>(&roll));

    yawMsg.data = yaw;
    pitchMsg.data = pitch;
    rollMsg.data = roll;

    yawPublisher.publish(yawMsg);
    pitchPublisher.publish(pitchMsg);
    rollPublisher.publish(rollMsg);
}
void ArduinoCommunication::onSpeed(uint8_t *message) {
    geometry_msgs::Twist twist;
    float result;
    std::copy(reinterpret_cast<const char *>(&message[1]),
              reinterpret_cast<const char *>(&message[5]),
              reinterpret_cast<char *>(&result));
    twist.linear.x = result;
    twistPublisher.publish(twist);

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
    std_msgs::UInt8 msg;
    uint8_t ticks;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[1]),
              reinterpret_cast<char *>(&ticks));

    msg.data = ticks;
    ticksPublisher.publish(msg);
}

void ArduinoCommunication::onSteeringCommand(std_msgs::UInt8 const &steering) {
    uint8_t size = sizeof(MessageType) + sizeof(uint8_t);
    uint8_t message[size];
    uint8_t output[size * 2 + 1];

    message[0] = (uint8_t) MessageType::STEERING_CMD;
    message[1] = steering.data;
    auto cobs = cobsEncode(message, size, output);

    onSend(output, cobs);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

void ArduinoCommunication::onLedCommand(std_msgs::String const &led) {
    uint8_t size = sizeof(MessageType) + led.data.length() + 2;
    uint8_t message[size];
    uint8_t output[size * 2 + 1];

    output[0] = (uint8_t) MessageType::LED_CMD;
    memcpy(&message[1], led.data.c_str(), led.data.length() + 1);
    auto cobs = cobsEncode(message, size, output);

    onSend(output, cobs);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }

}

void ArduinoCommunication::onSpeedCommand(std_msgs::Int16 const &speed) {
    uint8_t size = sizeof(MessageType) + sizeof(int16_t);
    uint8_t message[size];
    uint8_t output[size * 2 + 1];

    message[0] = (uint8_t) MessageType::SPEED_CMD;
    memcpy(&message[1], &speed.data, sizeof(uint16_t));
    auto cobs = cobsEncode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

}
