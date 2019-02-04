#include <nodelet/loader.h>
#include <ros/ros.h>

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
    ros::init(argc, argv, "emergency_stop_node");

    nodelet::Loader nodelet;
    const nodelet::M_string &remappings(ros::names::getRemappings());
    nodelet::V_string nodeletArgv(argv, argv + argc);

    std::string nodeletName = "emergency_stop/Nodelet";
    // nodelets_plugins.xml refers to the value of nodeletName as "name"
    if (not nodelet.load(ros::this_node::getName(), nodeletName, remappings, nodeletArgv)) {
        return -1;
    }

    ros::spin();
}
