#include <send_steering_light_speed.h>

class steering_light_control
{
  private:
    ros::Subscriber sub_steering_;
    ros::Subscriber sub_light_;
    ros::Subscriber sub_speed_,sub_stop_;
  public:

    steering_light_control(ros::NodeHandle nh)
    {
      
      sub_steering_ = nh.subscribe( "manual_control/steering", 1,  &steering_light_control::steeringCallback,this);
      sub_light_ = nh.subscribe( "manual_control/lights", 1,  &steering_light_control::lightCallback,this);
      sub_speed_ = nh.subscribe( "manual_control/speed", 1, &steering_light_control::motorSpeedCallback,this);
      sub_stop_ = nh.subscribe( "manual_control/stop_start", 1,  &steering_light_control::motorStopStartCallback,this);
      pub=false;
      speed=0;
      steering=90;
      
    }
    ~steering_light_control(){}
    void steeringCallback(const std_msgs::Int16 steering_value);
    void lightCallback(const std_msgs::String light_value);
    void motorSpeedCallback(const std_msgs::Int16 motor_value);
    void motorStopStartCallback(const std_msgs::Int16 stop_value);
    bool pub;
    std::string light;
    int steering;
    int speed;
    send_steering_light_speed steering_light_speed;
    
};

void steering_light_control::steeringCallback(const std_msgs::Int16 steering_value)
{ 
  steering=steering_value.data;
  pub=true;
}

void steering_light_control::lightCallback(const std_msgs::String light_value)
{ 
  light=light_value.data;
  pub=true;
}

void steering_light_control::motorSpeedCallback(const std_msgs::Int16 motor_value)
{ 
  speed=motor_value.data;
  pub=true;
}
void steering_light_control::motorStopStartCallback(const std_msgs::Int16 stop_value)
{ 
  if (stop_value.data==1)
  {
    speed=0;
    pub=true;
  } 
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "send_steering_light_speed_node");
  ros::NodeHandle nh;
  steering_light_control slc(nh);
  ros::Rate rate(1000);
   while(ros::ok())
  {
    if (slc.pub==true)
    {
      slc.steering_light_speed.run(slc.steering,slc.speed,slc.light);
      slc.pub=false;
    }
    ros::spinOnce();  
    rate.sleep();
  }
  slc.steering_light_speed.stop();
  return 0;
}
