#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

#include "fub_modelcar_tools/fub_modelcar_tools.h"
#include <vector>
using namespace fub_modelcar_tools;


double x = 0.0;
double y = 0.0;
double th = 0.0;

double steer_angle=0.0;
double head=0.0;
double last_head=0.0;
double initial_head=0.0;
double v = 0.0;
double vth = 0.0;
//Distance of IMU to the rear axis
double lr=0.24;
//Distance of IMU to the front axis
double lf=0.02;
bool init=false;
//std::vector<double> scale(180/30);//scal factor= 30
const double scale_factor=30;
std::vector <IO_pair> steeringPairs;

ros::Time current_time_twist, last_time_twist;

void twistCallback(const geometry_msgs::Twist& msg)
{
  float v_=round(msg.linear.x / (5.5))*(0.031);//rad/s and gear ratio: 5.5  and the wheel Radius 0.031 meter
  v = roundf(v_ * 100) / 100;  /* Result: 37.78 */
}
void headingCallback(const std_msgs::Float32& msg)
{
  if (init==false)
  {
    init=true;
    head=msg.data* (3.14/180.0); //rad
    initial_head=head;
    vth=0.0;
    current_time_twist = ros::Time::now();
    last_time_twist=current_time_twist;
    th=msg.data* (3.14/180.0);
  }
  else
  {
    // last_time_twist=current_time_twist;
    // current_time_twist = ros::Time::now();
    //last_head=head;
    head=msg.data* (3.14/180.0); //rad
    // double dt_twist = (current_time_twist - last_time_twist).toSec();
    double delta_head=head-initial_head;
    if (delta_head>3.14)
      delta_head=delta_head-6.28;
    else if (delta_head<-3.14)
      delta_head=delta_head+6.28;
    // float vth_=0.0;
    // vth_= delta_head/dt_twist;//100
    // vth = roundf(vth_ * 1000) / 1000;  /* Result: 37.78 */
    th = roundf(delta_head * 100) / 100;
  }
  
}
void steeringCallback(const std_msgs::Int16& msg)
{
    double steer=msg.data;
    int i=int(steer/scale_factor);
    if (steeringPairs.size()>=(i+1))
        steer_angle=((steer-i*scale_factor)*(steeringPairs.at(i+1).steering-steeringPairs.at(i).steering))/scale_factor+steeringPairs.at(i).steering;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  std::string file_name;
  n.param<std::string>("file_name",file_name,"/cfg/SteerAngleActuator.xml");
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Subscriber twist_sub = n.subscribe( "motor_control/twist", 10, twistCallback);
  ros::Subscriber theta_sub = n.subscribe( "model_car/yaw", 10, headingCallback);//degree
  ros::Subscriber steering_sub = n.subscribe( "manual_control/steering", 10, steeringCallback);//steering
  fub_modelcar_tools::restoreXML(steeringPairs,file_name.c_str());
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1000.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    dt = roundf(dt * 10000) / 10000;
    double beta=atan((lr/(lr+lf))*tan(steer_angle));
    double delta_x = (v * cos(th+beta)) * dt;
    double delta_y = (v * sin(th+beta)) * dt;

    vth =v * sin(beta)/lr ; //* dt;
    x += delta_x;
    y += delta_y;
    //th += delta_th;

    // ROS_INFO("t %0.6f",dt);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
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

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = v * cos(th+beta);
    odom.twist.twist.linear.y = v * sin(th+beta);
    odom.twist.twist.angular.z=vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
