#include <odometry/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/forwards.h>
#include <tf2/utils.h>

namespace odometry {
    Odometry::Odometry() : x(0), y(0), yaw(0) {}

    Odometry::~Odometry() = default;

    void Odometry::setConfig(odometry::OdometryConfig &config) {
        this->config = config;
    }

    void Odometry::setSpeed(const autominy_msgs::SpeedConstPtr &speed) {
        currentSpeed = speed->value;
    }

    void Odometry::setSteering(const autominy_msgs::SteeringAngleConstPtr &speed) {
        currentSteering = speed->value;
    }

    nav_msgs::Odometry Odometry::step(const ros::TimerEvent& evnt) {

        nav_msgs::Odometry odom;

        auto dt = (evnt.current_real - evnt.last_real).toSec();
        auto vth = (currentSpeed / config.axle_distance) * tan(currentSteering);
        yaw += vth * dt;
        x += currentSpeed * cos(yaw) * dt;
        y += currentSpeed * sin(yaw) * dt;

        odom.header.stamp = evnt.current_real;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
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
                                 0, 0, 0, 0, 0, 0.01
        };

        odom.twist.covariance = {0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.01
        };

        if (config.publish_tf) {
            geometry_msgs::TransformStamped trans;
            trans.header = odom.header;
            trans.child_frame_id = odom.child_frame_id;
            trans.transform.translation.x = odom.pose.pose.position.x;
            trans.transform.translation.y = odom.pose.pose.position.y;
            trans.transform.rotation = odom.pose.pose.orientation;

            tfBroadCaster.sendTransform(trans);
        }

        return odom;
    }

}
