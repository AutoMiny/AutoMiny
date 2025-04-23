#pragma once

#include "rclcpp/rclcpp.hpp"

#include <odometry/Odometry.h>
#include "tf2_ros/transform_broadcaster.h"
#include "autominy_msgs/msg/speed.hpp"
#include "autominy_msgs/msg/steering_angle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace odometry {

/** Odometry nodelet. Does nothing. You can break
 ** lines like this.
 **
 ** @ingroup @@
 */
    class OdometryNodelet : public rclcpp::Node {
    public:
        /** Destructor.
         */
        ~OdometryNodelet() override = default;

        /** Nodelet initialization. Called by nodelet manager on initialization,
         ** can be used to e.g. subscribe to topics and define publishers.
         */
        OdometryNodelet(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

    private:

        void onOdometry();

        void onSpeed(autominy_msgs::msg::Speed::ConstSharedPtr const &msg);

        void onSteering(autominy_msgs::msg::SteeringAngle::ConstSharedPtr const &msg);

        /// subscriber
        rclcpp::Subscription<autominy_msgs::msg::Speed>::SharedPtr speedSubscriber;
        rclcpp::Subscription<autominy_msgs::msg::SteeringAngle>::SharedPtr steeringSubscriber;

        /// publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryPublisher;

        rclcpp::TimerBase::SharedPtr timer;

        double axleDistance = 0.27;
        bool publishTf = false;

        double currentSpeed = 0.0;
        double currentSteering = 0.0;
        double x, y, yaw = 0.0;

        std::string baseLinkFrame;
        std::string odomFrame;

        rclcpp::Time last;

        tf2_ros::TransformBroadcaster tfBroadCaster;
    };
}

