#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringCommand.h>
#include <autominy_msgs/SteeringAngle.h>
#include <sensor_msgs/Imu.h>
#include <vector>

nav_msgs::Odometry ground_truth;

bool bicycle_model = false;
bool publish_tf = false;

bool init = false;

void groundTruthCallback(const nav_msgs::Odometry &msg) {
    ground_truth = msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n("~");

    n.param("bicycle_model", bicycle_model, false);
    n.param("publish_tf", publish_tf, false);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Subscriber ground_truth_sub = n.subscribe("/odom_ground_truth", 1, groundTruthCallback, ros::TransportHints().tcpNoDelay());

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;
    last_time = ros::Time::now();

    ros::Rate r(100.0);
    while (n.ok()) {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //first, we'll publish the transform over tf
        if (publish_tf) {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = ground_truth.pose.pose.position.x;
            odom_trans.transform.translation.y = ground_truth.pose.pose.position.y;
            odom_trans.transform.translation.z = 0;
            odom_trans.transform.rotation = ground_truth.pose.pose.orientation;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);
        }

        // we'll just forward the ground truth msgs
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = ground_truth.pose.pose.position.x;
        odom.pose.pose.position.y = ground_truth.pose.pose.position.y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = ground_truth.pose.pose.orientation;

        // covariance
        odom.pose.covariance = { 0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.01
        };
        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = ground_truth.twist.twist.linear.x;
        odom.twist.twist.linear.y = ground_truth.twist.twist.linear.y;
        odom.twist.twist.angular.z = 0;

        odom.twist.covariance = { 0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.01
        };
        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}
