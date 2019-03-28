#pragma once

#include <memory>

namespace tick_calibration {

/** Forward declaration of the TickCalibration class with type aliases.
 **
 ** If the TickCalibration class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
    class TickCalibration;

    using TickCalibrationPtr = std::shared_ptr<TickCalibration>;
    using TickCalibrationConstPtr = std::shared_ptr<const TickCalibration>;
}
