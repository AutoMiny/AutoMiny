#include "odometry/Odometry.h"

namespace odometry {
    OdometryNodelet::OdometryNodelet(const rclcpp::NodeOptions &opts) : rclcpp::Node("odometry", opts),
                                                                                  tfBroadCaster(this) {
        axleDistance = declare_parameter<double>("axle_distance", 0.27);
        publishTf = declare_parameter<bool>("publish_tf", false);
        baseLinkFrame = declare_parameter<std::string>("base_link_frame", "base_link");
        odomFrame = declare_parameter<std::string>("odom_frame", "odom");

        odometryPublisher = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
        speedSubscriber = create_subscription<autominy_msgs::msg::Speed>("speed", 1,
                                                                         std::bind(&OdometryNodelet::onSpeed, this,
                                                                                   std::placeholders::_1));
        steeringSubscriber = create_subscription<autominy_msgs::msg::SteeringAngle>("steering", 1, std::bind(
                &OdometryNodelet::onSteering, this, std::placeholders::_1));

        last = now();
        timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(0.01),
                                     std::bind(&OdometryNodelet::onOdometry, this));
    }

    void OdometryNodelet::onOdometry() {
        nav_msgs::msg::Odometry odom;

        auto t = now();
        auto dt = (t - last).seconds();
        last = t;
        if (dt <= 0) return;

        auto vth = (currentSpeed / axleDistance) * tan(currentSteering);
        yaw += vth * dt;
        x += currentSpeed * cos(yaw) * dt;
        y += currentSpeed * sin(yaw) * dt;

        odom.header.stamp = t;
        odom.header.frame_id = baseLinkFrame;
        odom.child_frame_id = odomFrame;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.twist.twist.linear.x = currentSpeed;
        odom.twist.twist.angular.z = vth;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom.pose.covariance = { 0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.03
        };

        odom.twist.covariance = {0.001, 0, 0, 0, 0, 0,
                                 0, 0.0001, 0, 0, 0, 0,
                                 0, 0, 0.001, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.03
        };

        if (publishTf) {
            geometry_msgs::msg::TransformStamped trans;
            trans.header = odom.header;
            trans.child_frame_id = odom.child_frame_id;
            trans.transform.translation.x = odom.pose.pose.position.x;
            trans.transform.translation.y = odom.pose.pose.position.y;
            trans.transform.rotation = odom.pose.pose.orientation;

            tfBroadCaster.sendTransform(trans);
        }

        odometryPublisher->publish(odom);
    }

    void OdometryNodelet::onSpeed(const autominy_msgs::msg::Speed::ConstSharedPtr &msg) {
        currentSpeed = msg->value;
    }

    void OdometryNodelet::onSteering(const autominy_msgs::msg::SteeringAngle::ConstSharedPtr &msg) {
        currentSteering = msg->value;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(odometry::OdometryNodelet)