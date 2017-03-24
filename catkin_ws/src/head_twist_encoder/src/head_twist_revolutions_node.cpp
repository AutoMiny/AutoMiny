#include <head_twist_revolutions/head_twist_revolutions.h>


int main(int argc, char **argv) 
{
  ros::init(argc, argv, "head_twist_revolutions_node");
  ros::NodeHandle nh;
  head_twist_revolutions ReadHeading(nh);
  ros::Rate rate(100);
   while(ros::ok())
  {
    ros::spinOnce();
    ReadHeading.get();
    rate.sleep();
  }
  return 0;
}
