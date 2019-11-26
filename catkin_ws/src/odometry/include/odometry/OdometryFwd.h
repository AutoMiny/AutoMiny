#pragma once

#include <memory>

namespace odometry {

/** Forward declaration of the Odometry class with type aliases.
 **
 ** If the Odometry class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
    class Odometry;

    using OdometryPtr = std::shared_ptr<Odometry>;
    using OdometryConstPtr = std::shared_ptr<const Odometry>;
}
