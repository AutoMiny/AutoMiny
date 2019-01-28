#pragma once

#include <memory>

namespace stereo_camera_pose_estimation {

/** Forward declaration of the Dummy class with type aliases.
 **
 ** If the Dummy class is pointed from another class without using its methods,
 ** then only this forward declaration is needed (saves compile time).
 **
 */
    class StereoCameraPoseEstimation;

    using StereoCameraPoseEstimationUniquePtr = std::unique_ptr<StereoCameraPoseEstimation>;
    using StereoCameraPoseEstimationUniqueConstPtr = std::unique_ptr<const StereoCameraPoseEstimation>;
    using StereoCameraPoseEstimationPtr = std::shared_ptr<StereoCameraPoseEstimation>;
    using StereoCameraPoseEstimationConstPtr = std::shared_ptr<const StereoCameraPoseEstimation>;
}
