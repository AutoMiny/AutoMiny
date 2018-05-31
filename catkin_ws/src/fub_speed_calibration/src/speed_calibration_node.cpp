#include <ros/ros.h>

#include "speed_calibration.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_calibration_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    fub_speed_calibration::SpeedCalibration calibration(nh);

	ros::spin();
}
