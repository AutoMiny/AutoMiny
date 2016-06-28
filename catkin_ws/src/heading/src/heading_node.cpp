#include <heading/heading.h>


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "heading_node");
  ros::NodeHandle nh;
  heading ReadHeading(nh);
   while(ros::ok())
  {
    ros::spinOnce();
    ReadHeading.getHeading();
    //ROS_INFO("yaw: %f",yaw);
    
  }
  return 0;
}
