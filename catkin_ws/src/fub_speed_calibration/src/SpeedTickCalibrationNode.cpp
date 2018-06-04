#include <ros/ros.h>

#include "SpeedTickCalibration.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_tick_calibration_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    fub_speed_calibration::SpeedTickCalibration calibration(nh_);

    ros::spin();
}
