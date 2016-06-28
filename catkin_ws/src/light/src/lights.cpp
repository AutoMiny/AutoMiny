#include <lights.h>

lights::lights() : priv_nh_("~"), lights_serial("/dev/ttyUSB3",115200, serial::Timeout::simpleTimeout(1000))
{
  result="";
  priv_nh_.param<std::string>("lights_serial_port", serial_port_, "/dev/ttyUSB3");
  priv_nh_.param("lights_baud_rate", baud_rate_,115200);
  lights_serial.close();
  lights_serial.setPort(serial_port_);
  lights_serial.setBaudrate(baud_rate_);
  lights_serial.open();
  init();
}

    //! Empty stub
lights::~lights() {}
void lights::init()
{
  	try
    {
      ROS_INFO("lights:: Is the arduino serial port %s open?",serial_port_.c_str());
      //cout << my_serial.getBaudrate() << endl;
  	  if(lights_serial.isOpen())
        ROS_INFO("lights::Yes.");
      else
        ROS_ERROR("lights::No.");
  	  bytes_wrote =lights_serial.write("enL\r");
    }
    catch(const std::exception& e)
    {	 
      ROS_ERROR("lights::could not find serial port");
    }
}
void lights::start()
{
    try
    {
      bytes_wrote =lights_serial.write("enL\r");
    }
    catch(const std::exception& e)
    {  
      ROS_ERROR("lights::could not find serial port");
    }
}
void lights::run(string light_direction)
{
  	try
    {
      string test_string=light_direction +"\r";
	    bytes_wrote =lights_serial.write(test_string);
    }
    catch(const std::exception& e)
    {	 
      	ROS_ERROR("lights::could not find serial port");
    }
}
void lights::stop()
{
	try
	{
		//serial::Serial my_serial(serial_port_, baud_rate_, serial::Timeout::simpleTimeout(1000));
		bytes_wrote =lights_serial.write("diL\r");
	}
	catch(const std::exception& e)
	{
		ROS_ERROR("lights::could not find serial port");
	}
}
