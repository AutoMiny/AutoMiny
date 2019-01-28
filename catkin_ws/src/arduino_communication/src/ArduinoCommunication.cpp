#include "ArduinoCommunication.h"

namespace arduino_communication {

ArduinoCommunication::ArduinoCommunication(ros::NodeHandle &nh) {
    device = nh.param<std::string>("device", "/dev/ttyUSB1");
    baudrate = nh.param("baud", 115200);

    speedSubscriber = nh.subscribe("speed", 2, &ArduinoCommunication::onSpeedCommand, this);
    steeringSubscriber = nh.subscribe("steering", 2, &ArduinoCommunication::onSteeringCommand, this);
    ledSubscriber = nh.subscribe("led", 2, &ArduinoCommunication::onLedCommand, this);

    twistPublisher = nh.advertise<geometry_msgs::Twist>("twist", 2);
    steeringAnglePublisher = nh.advertise<autominy_msgs::SteeringFeedback>("steering_angle", 2);
    voltagePublisher = nh.advertise<autominy_msgs::Voltage>("voltage", 2);
    ticksPublisher = nh.advertise<autominy_msgs::Tick>("ticks", 2);
    imuPublisher = nh.advertise<sensor_msgs::Imu>("imu", 2);
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

    autominy_msgs::SteeringFeedback msg;
    msg.value = angle;
    steeringAnglePublisher.publish(msg);

}
void ArduinoCommunication::onVoltage(uint8_t *message) {
    float voltage;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[4]),
              reinterpret_cast<char *>(&voltage));

    autominy_msgs::Voltage msg;
    msg.value = voltage;
    voltagePublisher.publish(msg);
}
void ArduinoCommunication::onIMU(uint8_t *message) {
    sensor_msgs::Imu imuMsg;
    imuMsg.header.frame_id = "imu";

    int16_t w = (((0xff &(char)message[0]) << 8) | 0xff &(char)message[1]);
    int16_t x = (((0xff &(char)message[2]) << 8) | 0xff &(char)message[3]);
    int16_t y = (((0xff &(char)message[4]) << 8) | 0xff &(char)message[5]);
    int16_t z = (((0xff &(char)message[6]) << 8) | 0xff &(char)message[7]);

    double wf = w/16384.0;
    double xf = x/16384.0;
    double yf = y/16384.0;
    double zf = z/16384.0;

    int16_t gx = (((0xff &(char)message[8]) << 8) | 0xff &(char)message[9]);
    int16_t gy = (((0xff &(char)message[10]) << 8) | 0xff &(char)message[11]);
    int16_t gz = (((0xff &(char)message[12]) << 8) | 0xff &(char)message[13]);

    double gxf = gx * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
    double gyf = gy * (4000.0/65536.0) * (M_PI/180.0) * 25.0;
    double gzf = gz * (4000.0/65536.0) * (M_PI/180.0) * 25.0;

    int16_t ax = (((0xff &(char)message[14]) << 8) | 0xff &(char)message[15]);
    int16_t ay = (((0xff &(char)message[16]) << 8) | 0xff &(char)message[17]);
    int16_t az = (((0xff &(char)message[18]) << 8) | 0xff &(char)message[19]);
    // calculate accelerations in m/s²
    double axf = ax * (8.0 / 65536.0) * 9.81;
    double ayf = ay * (8.0 / 65536.0) * 9.81;
    double azf = az * (8.0 / 65536.0) * 9.81;

    int16_t temperature = (((0xff &(char)message[20]) << 8) | 0xff &(char)message[21]);
    double temperature_in_C = (temperature / 340.0 ) + 36.53;
    ROS_DEBUG_STREAM("Temperature [in C] " << temperature_in_C);

    // map from NED to ENU
    imuMsg.orientation.x = -xf;
    imuMsg.orientation.y = -yf;
    imuMsg.orientation.z = -zf;
    imuMsg.orientation.w = wf;

    imuMsg.angular_velocity.x = gyf;
    imuMsg.angular_velocity.y = -gxf;
    imuMsg.angular_velocity.z = -gzf;

    imuMsg.linear_acceleration.x = ayf;
    imuMsg.linear_acceleration.y = -axf;
    imuMsg.linear_acceleration.z = -azf;

    imuPublisher.publish(imuMsg);
}
void ArduinoCommunication::onSpeed(uint8_t *message) {
    geometry_msgs::Twist twist;
    float result;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[4]),
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
    autominy_msgs::Tick msg;
    uint8_t ticks;
    std::copy(reinterpret_cast<const char *>(&message[0]),
              reinterpret_cast<const char *>(&message[1]),
              reinterpret_cast<char *>(&ticks));

    msg.value = ticks;
    ticksPublisher.publish(msg);
}

void ArduinoCommunication::onSteeringCommand(autominy_msgs::SteeringCommandConstPtr const &steering) {
    uint8_t size = sizeof(MessageType) + sizeof(int16_t);
    uint8_t message[size];
    uint8_t output[size * 2 + 1];

    message[0] = (uint8_t) MessageType::STEERING_CMD;
    memcpy(&message[1], &steering->value, sizeof(int16_t));
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

void ArduinoCommunication::onSpeedCommand(autominy_msgs::SpeedCommandConstPtr const &speed) {
    uint8_t size = sizeof(MessageType) + sizeof(int16_t);
    uint8_t message[size];
    uint8_t output[size * 2 + 1];

    message[0] = (uint8_t) MessageType::SPEED_CMD;
    memcpy(&message[1], &speed->value, sizeof(int16_t));
    auto cobs = cobsEncode(message, size, output);

    auto wrote = onSend(output, cobs);
    if (wrote != cobs) {
        ROS_ERROR("Could not write all of the data. Size should be %lu but wrote only %lu", cobs, wrote);
    }
}

}
