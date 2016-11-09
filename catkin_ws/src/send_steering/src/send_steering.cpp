#include <send_steering.h>

send_steering::send_steering() : priv_nh_("~"), servo_serial("/dev/ttyUSB3",115200, serial::Timeout::simpleTimeout(1000))
{
  result="";
  priv_nh_.param<std::string>("servo_serial_port", serial_port_, "/dev/ttyUSB3");
  priv_nh_.param("servo_baud_rate", baud_rate_,115200);
  servo_serial.close();
  servo_serial.setPort(serial_port_);
  servo_serial.setBaudrate(baud_rate_);
  servo_serial.open();
  init();
}

    //! Empty stub
send_steering::~send_steering() {}
void send_steering::init()
{
  	try
    {
      ROS_INFO("send_steering:: Is the servo serial port %s open?",serial_port_.c_str());
      //cout << my_serial.getBaudrate() << endl;
  	  if(servo_serial.isOpen())
        ROS_INFO("send_steering::Yes.");
      else
        ROS_ERROR("send_steering::No.");
  	  bytes_wrote =servo_serial.write("en\r");
    }
    catch(const std::exception& e)
    {	 
      ROS_ERROR("send_steering::could not find serial port");
    }
}
void send_steering::start()
{
    try
    {
      bytes_wrote =servo_serial.write("en\r");
    }
    catch(const std::exception& e)
    {  
      ROS_ERROR("send_steering::could not find serial port");
    }
}
void send_steering::run(int speed)
{
  	try
    {
      std::string speed_string = std::to_string(speed);;
	    string test_string=speed_string +"\r";
	    bytes_wrote =servo_serial.write(test_string);
    }
    catch(const std::exception& e)
    {	 
      	ROS_ERROR("send_steering::could not find serial port");
    }
}
void send_steering::stop()
{
	try
	{
		//serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
		bytes_wrote =servo_serial.write("di\r");
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("send_steering::could not find serial port");
	}
}
