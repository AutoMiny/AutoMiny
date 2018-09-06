#pragma once

#include <cstdint>

enum class MessageType : uint8_t {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    SPEED_CMD,
    STEERING_CMD,
    LED_CMD,
    STEERING_ANGLE,
    TICKS,
    SPEED,
    IMU,
    VOLTAGE
};