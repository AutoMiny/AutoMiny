#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <autominy_msgs/Speed.h>
#include <autominy_msgs/SteeringAngle.h>
#include <sensor_msgs/Imu.h>
#include <vector>

double x = 0.0;
double y = 0.0;
double initial_x = 0.0;
double initial_y = 0.0;
double initial_yaw = 0.0;
double th = 0.0;
bool bicycle_model = false;
bool publish_tf = false;

double steer_angle = 0.0;
double head = 0.0;
double initial_head = 0.0;
double v = 0.0;
double vth = 0.0;
//Distance of IMU to the rear axis
double lr = 0.03;
//Distance of IMU to the front axis
double lf = 0.23;
bool init = false;

void twistCallback(const autominy_msgs::Speed &msg) {
    v = msg.value;
}

void headingCallback(const sensor_msgs::Imu &msg) {
    auto yaw = tf::getYaw(msg.orientation);
    if (std::isnan(yaw)) {
        return;
    }

    head = yaw;

    if (!init) {
        init = true;
        initial_head = head;
        vth = 0.0;
        th = initial_yaw;
    } else {
        double delta_head = head - initial_head + initial_yaw;
        if (delta_head > 3.14)
            delta_head = delta_head - 6.28;
        else if (delta_head < -3.14)
            delta_head = delta_head + 6.28;
        //the yaw should increase clock wise -pi< yaw < pi
        th = delta_head;
    }
}

void steeringFeedbackCallback(const autominy_msgs::SteeringAngleConstPtr &msg) {
    steer_angle = msg->value;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle n("~");

    std::string model_car_twist, model_car_yaw, steering_command, steering_feedback;
    n.param<std::string>("model_car_twist", model_car_twist, "speed");
    n.param<std::string>("model_car_yaw", model_car_yaw, "imu");
    n.param<std::string>("steering_feedback", steering_feedback, "steering");

    n.param("initial_x", initial_x, 2.5);
    n.param("initial_y", initial_y, 0.5);
    n.param("initial_yaw", initial_yaw, 0.0);
    n.param("bicycle_model", bicycle_model, false);
    n.param("publish_tf", publish_tf, false);

    x = initial_x;
    y = initial_y;
    th = initial_yaw;

    ROS_INFO_STREAM("initial_x:" << initial_x << " m, initial_y: " << initial_y << " m, initial_yaw: " << initial_yaw
                                 << " radians");

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    ros::Subscriber twist_sub = n.subscribe(model_car_twist, 1, twistCallback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber theta_sub = n.subscribe(model_car_yaw, 1, headingCallback, ros::TransportHints().tcpNoDelay());//degree
    ros::Subscriber steering_sub;
    steering_sub = n.subscribe(steering_feedback, 1, steeringFeedbackCallback, ros::TransportHints().tcpNoDelay());//steering feedback

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;
    last_time = ros::Time::now();

    ros::Rate r(100.0);
    while (n.ok()) {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();

        if (dt > 1 || dt < 0.001) {
            ROS_ERROR("Odometry time difference too small or big");
            last_time = current_time;
            r.sleep();
            continue;
        }

        double beta = atan((lr / (lr + lf)) * tan(steer_angle));
        double delta_x = 0.0;
        double delta_y = 0.0;

        if (bicycle_model) {
            delta_x = (v * cos(th + beta)) * dt; //v unit = mm/s -> output : m
            delta_y = (v * sin(th + beta)) * dt;
            x += delta_x;
            y += delta_y;
        } else {
            delta_x = (v * cos(th)) * dt; //v unit = mm/s -> output : m
            delta_y = (v * sin(th)) * dt;
            x += delta_x;
            y += delta_y;
        }


        vth = v * sin(beta) / lr; //* dt;
        // x+=delta_x;
        // y+=delta_y;
        // th += vth * dt;

        //a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        if (publish_tf) {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";
            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            //send the transform
            odom_broadcaster.sendTransform(odom_trans);
        }

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

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
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = vth;

        odom.twist.covariance = {0.01, 0, 0, 0, 0, 0,
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
