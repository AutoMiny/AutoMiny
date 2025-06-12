#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "bno055_usb_stick/bno055_usb_stick.hpp"
#include "bno055_usb_stick/decoder.hpp"
#include "bno055_usb_stick_msgs/msg/output.hpp"

#include <boost/asio/io_service.hpp>
#include <boost/shared_ptr.hpp>

namespace bus = bno055_usb_stick;

rclcpp::Publisher<bno055_usb_stick_msgs::msg::Output>::SharedPtr out_pub;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
std::string pose_frame_id;
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;
rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub;
std::string tf_frame_id, tf_child_frame_id;
bool invert_tf;

void publish(const bno055_usb_stick_msgs::msg::Output& output) {
    if (out_pub->get_subscription_count() > 0) {
        out_pub->publish(output);
    }
    if (imu_pub->get_subscription_count() > 0) {
        imu_pub->publish(bus::Decoder::toImuMsg(output));
    }
    if (pose_pub->get_subscription_count() > 0) {
        pose_pub->publish(bus::Decoder::toPoseMsg(output, pose_frame_id));
    }
    if (mag_pub->get_subscription_count() > 0) {
        mag_pub->publish(bus::Decoder::toMagMsg(output));
    }
    if (temp_pub->get_subscription_count() > 0) {
        temp_pub->publish(bus::Decoder::toTempMsg(output));
    }
    if (tf_pub) {
        tf_pub->sendTransform(
                bus::Decoder::toTFTransform(output, tf_frame_id, tf_child_frame_id, invert_tf));
    }
}

int main(int argc, char* argv[]) {
    // init ROS
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node("bno055_usb_stick_node");

    // load parameters
    pose_frame_id = nh.declare_parameter<std::string>("~pose_frame_id", "fixed");
    const bool publish_tf= nh.declare_parameter<bool>("~publish_tf", false);
    tf_frame_id = nh.declare_parameter<std::string>("~tf_frame_id", "fixed");
    tf_child_frame_id = nh.declare_parameter<std::string>("~tf_child_frame_id", "bno055");
    invert_tf = nh.declare_parameter<bool>("~invert_tf", false);

    // setup publishers
    out_pub = nh.create_publisher<bno055_usb_stick_msgs::msg::Output>("output", 1);
    imu_pub = nh.create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    pose_pub = nh.create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
    mag_pub = nh.create_publisher<sensor_msgs::msg::MagneticField>("magnetic_field", 1);
    temp_pub = nh.create_publisher<sensor_msgs::msg::Temperature>("temperature", 1);

    if (publish_tf) {
        tf_pub.reset(new tf2_ros::TransformBroadcaster(nh));
    }

    // construct the worker
    boost::asio::io_service asio_service;
    bus::BNO055USBStick device(nh, asio_service, publish);

    // run the worker
    while (rclcpp::ok()) {
        asio_service.run_one();
    }

    return 0;
}