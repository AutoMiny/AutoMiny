#include <send_steering_light_speed.h>

send_steering_light_speed::send_steering_light_speed() : priv_nh_("~"), arduino_serial("/dev/ttyUSB1",115200, serial::Timeout::simpleTimeout(1000))
{
  result="";
  priv_nh_.param<std::string>("arduino_serial_port", serial_port_, "/dev/ttyUSB3");
  priv_nh_.param("arduino_baud_rate", baud_rate_,115200);
  arduino_serial.close();
  arduino_serial.setPort(serial_port_);
  arduino_serial.setBaudrate(baud_rate_);
  arduino_serial.open();
  init();
}

    //! Empty stub
send_steering_light_speed::~send_steering_light_speed() {}
void send_steering_light_speed::init()
{
  	try
    {
      ROS_INFO("send_steering_light_speed:: Is the servo serial port %s open?",serial_port_.c_str());
      //cout << my_serial.getBaudrate() << endl;
  	  if(arduino_serial.isOpen())
        ROS_INFO("send_steering_light_speed::Yes.");
      else
        ROS_ERROR("send_steering_light_speed::No.");
      bytes_wrote =arduino_serial.write("SenLdiLM0\r");
    }
    catch(const std::exception& e)
    {	 
      ROS_ERROR("send_steering_light_speed::could not find serial port");
    }
}
void send_steering_light_speed::start()
{
    try
    {
      bytes_wrote =arduino_serial.write("SenLdiLM0\r");
    }
    catch(const std::exception& e)
    {  
      ROS_ERROR("send_steering_light_speed::could not find serial port");
    }
}
void send_steering_light_speed::run(int steering,int speed,std::string light_string)
{
  	try
    {
      std::string steering_string = std::to_string(180-steering);
      std::string speed_string = std::to_string(speed);
	    string test_string="S"+steering_string +"L"+light_string+"M"+speed_string+"\r";
      //ROS_INFO("send_steering_light_speed::%s",test_string.c_str());
	    bytes_wrote =arduino_serial.write(test_string);
    }
    catch(const std::exception& e)
    {	 
      ROS_ERROR("send_steering_light_speed::could not find serial port");
    }
}
void send_steering_light_speed::stop()
{
	try
	{
		//serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
		bytes_wrote =arduino_serial.write("SdiLdiLM00\r");
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("send_steering_light_speed::could not find serial port");
	}
}
