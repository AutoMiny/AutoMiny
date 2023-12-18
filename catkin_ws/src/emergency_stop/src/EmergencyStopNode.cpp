#include "emergency_stop/EmergencyStop.h"
#include "rclcpp/rclcpp.hpp"

/** Starting point for the node. It instantiates the nodelet within the node
 ** (alternatively the nodelet could be run in a standalone nodelet manager).
 **
 ** @param argc
 ** @param argv
 ** @return
 **
 ** @ingroup @@
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<emergency_stop::EmergencyStopNodelet>());
    rclcpp::shutdown();
}
