#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "serial/serial.h"


class heading
{
  private:
     //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    ros::Publisher pub_yaw_;

    std::string serial_port_;
    int baud_rate_;
    std::string result;
    size_t bytes_wrote;
    serial::Serial my_serial;

  public:
    
    heading(ros::NodeHandle nh);
    ~heading();
    void init();
    void getHeading();

};

