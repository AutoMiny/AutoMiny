#pragma once

#include <memory>

namespace obstacle_detection {

/** Forward declaration of the Dummy class with type aliases.
 **
 ** If the Dummy class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
    class ObstacleDetection;

    using ObstacleDetectionUniquePtr = std::unique_ptr<ObstacleDetection>;
    using ObstacleDetectionUniqueConstPtr = std::unique_ptr<const ObstacleDetection>;
    using ObstacleDetectionPtr = std::shared_ptr<ObstacleDetection>;
    using ObstacleDetectionConstPtr = std::shared_ptr<const ObstacleDetection>;
}
