#include <ros/ros.h>

#include "ArduinoCommunication.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arduino_communication_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    arduino_communication::ArduinoCommunication communication(nh_);
    communication.spin();
}
