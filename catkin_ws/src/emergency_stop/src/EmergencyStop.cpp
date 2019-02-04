#include <emergency_stop/EmergencyStop.h>

namespace emergency_stop {
    EmergencyStop::EmergencyStop() = default;

    EmergencyStop::~EmergencyStop() = default;

    void EmergencyStop::setConfig(emergency_stop::EmergencyStopConfig & config) { this->config = config; }

    void EmergencyStop::checkEmergencyStop(const sensor_msgs::LaserScanConstPtr &scan) {
        int count = static_cast<int>(scan->scan_time / scan->time_increment);
        double breakDistance= config.break_distance;
        if (std::abs(currentSpeed) > 50 && config.break_distance_based_on_speed) {
            breakDistance = (std::abs(currentSpeed) / 50) * config.break_distance;
        }

        if(currentSpeed > 0){	//forward.
            for(int i = 0; i < config.angle_front / 2 + 1; i++){
                if (scan->ranges[i] <= breakDistance + config.forward_minimum_distance && scan->ranges[i] > config.forward_minimum_distance){
                    emergencyStop = true;
                    return;
                }
            }
            for(int k = 360 - config.angle_front / 2; k < count; k++){
                if (scan->ranges[k] <= breakDistance + config.forward_minimum_distance && scan->ranges[k] > config.forward_minimum_distance){
                    emergencyStop = true;
                    return;
                }
            }
        }

        if(currentSpeed < 0){ //backward.
            for(int j = (180-(config.angle_back / 2)); j < (180 + config.angle_back / 2) + 1; j++){
                // we might see the camera in the laser scan
                if (scan->ranges[j] <= (breakDistance + config.reverse_minimum_distance) && scan->ranges[j] > config.reverse_minimum_distance){
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

    bool EmergencyStop::isEmergencyStop() {
        return emergencyStop;
    }
}
