#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"
#include <sstream>
#include <ros/console.h>
#include <sstream> 

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


class lights
{
  private:
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    std::string serial_port_;
    int baud_rate_;
    std::string result;
    size_t bytes_wrote;
    serial::Serial lights_serial;
    
  public:
    lights();
    ~lights();
    void init();
    void run(string light_direction);
    void stop();
    void start();
    
};

