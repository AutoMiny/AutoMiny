#pragma once

#include <memory>

namespace lidar_pose_estimation {

/** Forward declaration of the Dummy class with type aliases.
 **
 ** If the Dummy class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
    class LidarPoseEstimation;

    using LidarPoseEstimationUniquePtr = std::unique_ptr<LidarPoseEstimation>;
    using LidarPoseEstimationUniqueConstPtr = std::unique_ptr<const LidarPoseEstimation>;
    using LidarPoseEstimationPtr = std::shared_ptr<LidarPoseEstimation>;
    using LidarPoseEstimationConstPtr = std::shared_ptr<const LidarPoseEstimation>;
}
