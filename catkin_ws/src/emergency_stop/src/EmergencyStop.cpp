#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig &config) { this->config = config; }

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

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {

        double breakDistance = getBreakDistance();

        auto angleIncrement = scan->angle_increment;

        bool planIsToMoveForward = wantedSpeed >= 0;

        bool emergencyStopIsNecessary;

        if (planIsToMoveForward) {

            emergencyStopIsNecessary = movingForwardEvaluation(scan, breakDistance, angleIncrement);

            if (emergencyStopIsNecessary) {
                emergencyStop = true;
                return;
            }

        } else {

            emergencyStopIsNecessary = movingBackwardEvaluation(scan, breakDistance, angleIncrement);

            if (emergencyStopIsNecessary) {
                emergencyStop = true;
                return;
            }
        }

        emergencyStop = false;
    }

    double EmergencyStop::getBreakDistance() {
        double breakDistance = config.break_distance;
        if (config.break_distance_based_on_speed) {
            breakDistance = calculateBreakDistance();
        }
        return breakDistance;
    }

    double EmergencyStop::calculateBreakDistance() const {
        return std::pow(currentSpeed, 2) / 2.0 * config.negative_acceleration;
    }

    bool EmergencyStop::movingForwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan,
                                                double breakDistance, float angleIncrement) {

        double angleFront = config.angle_front / 2.0;

        bool emergencyStopIsNecessary = firstForwardEvaluation(scan, breakDistance, angleIncrement, angleFront);

        if (emergencyStopIsNecessary) {
            return true;
        }

        emergencyStopIsNecessary = secondForwardEvaluation(scan, breakDistance, angleIncrement, angleFront);

        return emergencyStopIsNecessary;

    }

    bool EmergencyStop::firstForwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan, double breakDistance,
                                               float angleIncrement, double angleFront) const {

        auto start = 0;
        auto end = static_cast<int>(angleFront / angleIncrement);

        for (int i = start; i < scan->ranges.size() && i < end; i++) {
            if (scan->ranges[i] <= breakDistance + config.forward_minimum_distance &&
                scan->ranges[i] > config.forward_minimum_distance) {
                return true;
            }
        }

        return false;
    }

    bool EmergencyStop::secondForwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan, double breakDistance,
                                                float angleIncrement, double angleFront) const {

        auto start = scan->ranges.size() - 1 - static_cast<int>(angleFront / angleIncrement);
        auto end = scan->ranges.size();

        for (int k = start; k < end; k++) {
            if (scan->ranges[k] <= breakDistance + config.forward_minimum_distance &&
                scan->ranges[k] > config.forward_minimum_distance) {
                return true;
            }
        }

        return false;
    }

    bool EmergencyStop::movingBackwardEvaluation(const sensor_msgs::LaserScanConstPtr &scan, double breakDistance,
                                                 float angleIncrement) {
        auto backAngle = config.angle_back / 2.0;
        int start = scan->ranges.size() / 2 - static_cast<int>(backAngle / angleIncrement);
        int end = scan->ranges.size() / 2 + static_cast<int>(backAngle / angleIncrement);

        for (int j = start; j < end && j < scan->ranges.size(); j++) {
            // we might see the camera in the laser scan
            if (scan->ranges[j] <= (breakDistance + config.reverse_minimum_distance) &&
                scan->ranges[j] > config.reverse_minimum_distance) {

                return true;
            }
        }
    }

}
