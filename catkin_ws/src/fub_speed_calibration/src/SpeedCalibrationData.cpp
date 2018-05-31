#include "SpeedCalibrationData.h"

namespace fub_speed_calibration {

SpeedCalibrationData::SpeedCalibrationData(int command, double twist) {
    this->command = command;
    this->twist = twist;
}

}