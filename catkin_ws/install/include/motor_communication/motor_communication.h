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


class motor_communication
{
  private:
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;
    std::string serial_port_;//="/dev/ttySAC2";
    int baud_rate_;//=115200;
    std::string result;
    size_t bytes_wrote;
    

    serial::Serial my_serial;
   
    //serial::Serial my_serial;
    //my_serial(serial_port_, 115200, serial::Timeout::simpleTimeout(1000)); 

  public:
    

    motor_communication();
    ~motor_communication();
    void init();
    void run(int speed);
    void my_sleep(unsigned long milliseconds);
    void stop();
    void start();
    double getSpeed();

};

