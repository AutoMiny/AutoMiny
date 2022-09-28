#include "rclcpp/rclcpp.hpp"

#include "ArduinoCommunication.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    arduino_communication::ArduinoCommunication communication;
    communication.spin();
    rclcpp::shutdown();
}
