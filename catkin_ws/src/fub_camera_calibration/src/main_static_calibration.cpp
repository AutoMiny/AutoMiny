#include <ros/ros.h>
#include <fub_camera_calibration/FubCameraStaticCalibration.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fub_camera_static_calibration");
    ros::NodeHandle nh;
    FubCameraStaticCalibration static_calib(nh);
    ros::spin();
    ros::shutdown();
    return 0;
}


// #include <ros/ros.h>
// #include <nodelet/loader.h>
// /** Starting point for the node. It instantiates the nodelet within the node
//  ** (alternatively the nodelet could be run in a standalone nodelet manager).
//  **
//  ** @param argc
//  ** @param argv
//  ** @return
//  **
//  ** @ingroup @@
//  */
// int main(int argc, char ** argv)
// {
//     ros::init(argc, argv, "fub_camera_static_calibration");

// 	nodelet::Loader nodelet;
// 	nodelet::M_string remappings(ros::names::getRemappings());
// 	nodelet::V_string nodeletArgv(argv, argv + argc);

//     std::string nodeletName = "fub_camera_static_calibration/Nodelet";
// 	if (not nodelet.load(ros::this_node::getName(), nodeletName, remappings, nodeletArgv)) {
// 		return -1;
// 	}

// 	ros::spin();
// }

