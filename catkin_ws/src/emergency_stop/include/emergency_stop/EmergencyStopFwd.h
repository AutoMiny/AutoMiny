#pragma once

#include <memory>

namespace emergency_stop {

/** Forward declaration of the EmergencyStop class with type aliases.
 **
 ** If the EmergencyStop class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
class EmergencyStop;

using EmergencyStopPtr = std::shared_ptr<EmergencyStop>;
using EmergencyStopConstPtr = std::shared_ptr<const EmergencyStop>;
}
