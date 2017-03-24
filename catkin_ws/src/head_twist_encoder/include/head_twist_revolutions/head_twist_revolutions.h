#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "serial/serial.h"
#include <geometry_msgs/Twist.h>



class head_twist_revolutions
{
  private:
     //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    ros::Publisher pub_yaw_;
    ros::Publisher pub_revolutions_;
    ros::Publisher pub_velocity_;

    std::string serial_port_;
    int baud_rate_;
    std::string result;
    size_t bytes_wrote;
    serial::Serial my_serial;
    float last_revolutions;

  public:
    
    head_twist_revolutions(ros::NodeHandle nh);
    ~head_twist_revolutions();
    void init();
    void get();

};

