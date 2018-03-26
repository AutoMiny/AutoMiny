#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tf/transform_broadcaster.h>
#include "nav_msgs/Odometry.h"
#include "spline.h"
#include "visualization.h"
#include "geometry_msgs/PoseStamped.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
ros::Time init_time;
int once =0;

void speed_callback(const std_msgs::Int16 &msg){
        init_time = ros::Time::now();
        once =1;
}

void odometryCallback(const nav_msgs::OdometryConstPtr & msg)
{
    double speed =(double) msg->twist.twist.linear.x;
    if(speed>0.6 & once ==1){
      
      ROS_INFO_STREAM((msg->header.stamp - init_time).toSec());
      ROS_INFO_STREAM((ros::Time::now() - init_time).toSec());
      once =0;
   }
     
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy");
  ros::NodeHandle nh;
  ros::Subscriber odom_pub = nh.subscribe("/odom", 1, odometryCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber speed_val = nh.subscribe("/manual_control/speed",1,speed_callback, ros::TransportHints().tcpNoDelay());
  ros::spin();
  return 0;
}

