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
typedef int16_t speed_MMpS_t;


class send_steering
{
  private:
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    std::string serial_port_;
    int baud_rate_;
    std::string result;
    size_t bytes_wrote;
    serial::Serial servo_serial;
    
  public:
    send_steering();
    ~send_steering();
    void init();
    void run(int speed);
    void stop();
    void start();
    
};

