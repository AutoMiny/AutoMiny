#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig &config) { this->config = config; }

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {
        double breakDistance = config.break_distance;
        if (config.break_distance_based_on_speed) {
            breakDistance = std::pow(currentSpeed, 2) / 2.0 * config.negative_acceleration;
        }

        auto angleIncrement = scan->angle_increment;
        if (wantedSpeed >= 0) {    //forward.
            auto frontAngle = config.angle_front / 2.0;
            auto start = 0;
            auto end = static_cast<int>(frontAngle / angleIncrement);

            for (int i = 0; i < scan->ranges.size() && i < end; i++) {
                if (scan->ranges[i] <= breakDistance + config.forward_minimum_distance &&
                    scan->ranges[i] > config.forward_minimum_distance) {
                    emergencyStop = true;
                    return;
                }
            }

            start = scan->ranges.size() - 1 - static_cast<int>(frontAngle / angleIncrement);
            end = scan->ranges.size();
            for (int k = start; k < end; k++) {
                if (scan->ranges[k] <= breakDistance + config.forward_minimum_distance &&
                    scan->ranges[k] > config.forward_minimum_distance) {
                    emergencyStop = true;
                    return;
                }
            }
        }

        if (wantedSpeed < 0) { //backward.
            auto backAngle = config.angle_back / 2.0;
            int start = scan->ranges.size() / 2 - static_cast<int>(backAngle / angleIncrement);
            int end = scan->ranges.size() / 2 + static_cast<int>(backAngle / angleIncrement);
            for (int j = start; j < end && j < scan->ranges.size(); j++) {
                // we might see the camera in the laser scan
                if (scan->ranges[j] <= (breakDistance + config.reverse_minimum_distance) &&
                    scan->ranges[j] > config.reverse_minimum_distance) {
                    emergencyStop = true;
                    return;
                }
            }
        }
        emergencyStop = false;
    }

    void EmergencyStop::setCurrentSpeed(const autominy_msgs::SpeedConstPtr &speed) {
        currentSpeed = speed->value;
    }

    void EmergencyStop::setWantedSpeed(const autominy_msgs::SpeedCommandConstPtr &speed) {
        wantedSpeed = speed->value;
    }

    autominy_msgs::SpeedCommand EmergencyStop::getSafeSpeed() {
        autominy_msgs::SpeedCommand msg;

        if (emergencyStop) {
            msg.value = 0;
        } else {
            msg.value = wantedSpeed;
        }

        return msg;
    }
}
