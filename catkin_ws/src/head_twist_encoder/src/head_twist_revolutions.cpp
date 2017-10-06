#include <head_twist_revolutions/head_twist_revolutions.h>

head_twist_revolutions::head_twist_revolutions(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"), my_serial("/dev/ttyUSB0",115200, serial::Timeout::simpleTimeout(1000))
{
  result="";
  priv_nh_.param<std::string>("arduino_serial_port", serial_port_, "/dev/ttyUSB0");
  priv_nh_.param("arduino_baud_rate", baud_rate_,115200);
  my_serial.close();
  my_serial.setPort(serial_port_);
  my_serial.setBaudrate(baud_rate_);
  my_serial.open();
  //my_serial.setTimeout(1000);
  //my_serial.Timeout.simpleTimeout(1000)
  pub_yaw_=nh_.advertise<std_msgs::Float32>(nh_.resolveName("model_car/yaw"), 1);
  pub_revolutions_=nh_.advertise<std_msgs::Float32>(nh_.resolveName("model_car/revolutions"), 1);
  pub_velocity_=nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("model_car/twist"), 1);
  manual_speed_sub = nh_.subscribe( "/manual_control/speed", 10, &head_twist_revolutions::directionCallback,this);//
  direction=1.0;
  init();
  manual_speed=0;
  last_revolutions=0.0;
  currentTwist.linear.x=0.0;
  currentTwist.linear.y = 0.0;
  currentTwist.linear.z = 0.0;
  
}
    //! Empty stub
head_twist_revolutions::~head_twist_revolutions() {}
void head_twist_revolutions::init()
{
    try
    {
      ROS_INFO("head_twist_revolutions::Is the serial port %s open?",serial_port_.c_str());
      //cout << my_serial.getBaudrate() << endl;
    if(my_serial.isOpen())
      ROS_INFO("head_twist_revolutions:: Yes.");
    else
      ROS_INFO("head_twist_revolutions:: No.");
    }
    catch(const std::exception& e)
    {  
        ROS_ERROR("head_twist_revolutions:: could not find serial port");
    }
}
void head_twist_revolutions::directionCallback(const std_msgs::Int16& msg)
{
       manual_speed=msg.data;
}
void head_twist_revolutions::get()
{
  result = "";
  try
  {
    // read until new line terminates result string
    uint8_t c = '\0';
    while (c  != '\n') {
      my_serial.read(&c, 1);
      result += char(c);
    }
    std::string angle_str = result.substr(1, result.find("s")-1); 
    result=result.substr(result.find("s")+1); 
    std::string time_str = result.substr(0,result.find("e")); 
    result=result.substr(result.find("e")+1); 
    std::string encoder_str = result.substr(0, result.find("\n"));

    std::stringstream ss(angle_str);
    std_msgs::Float32 angle;
    ss >> angle.data; //string to double
    pub_yaw_.publish(angle);

    std::stringstream ss2(encoder_str);
    std_msgs::Float32 revolutions;
    ss2 >> revolutions.data; //string to double
    revolutions.data=revolutions.data/6;
    pub_revolutions_.publish(revolutions);

    std::stringstream ss1(time_str);
    float delta_time;
    ss1 >> delta_time;
    if (delta_time!=0.0)
      currentTwist.linear.x=(3.14/3)/(delta_time*0.005*0.001); //rad/second -> each tick is 0.005 ms: Arduino timer is 2Mhz , but counter divided by 10 in arduino! 6 lines per revoloution!
    else
      currentTwist.linear.x=0.0;
     
    if (currentTwist.linear.x<((double)(std::abs(manual_speed))/5.0))
    if (manual_speed>0)
    	direction=-1.0;
    else if (manual_speed<0)
	direction=1.0;
    currentTwist.linear.x=currentTwist.linear.x*direction;
    currentTwist.linear.y = 0.0;
    currentTwist.linear.z = 0.0;
    pub_velocity_.publish(currentTwist);
    
    last_revolutions=revolutions.data;
  }
  catch(const std::exception& e)
  {  
    ROS_ERROR("head_twist_revolutions:: could not find serial port");
  }

}
