#include "rclcpp/rclcpp.hpp"
#include "stereo_camera_pose_estimation/StereoCameraPoseEstimation.h"

/** Starting point for the node. It instantiates the nodelet within the node
 ** (alternatively the nodelet could be run in a standalone nodelet manager).
 **
 ** @param argc
 ** @param argv
 ** @return
 **
 ** @ingroup @@
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stereo_camera_pose_estimation::StereoCameraPoseEstimationNodelet>());
    rclcpp::shutdown();
}
