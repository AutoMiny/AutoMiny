#include <send_steering_light.h>

class steering_light_control
{
  private:
    ros::Subscriber sub_steering_;
    ros::Subscriber sub_light_;
    
  public:

    steering_light_control(ros::NodeHandle nh)
    {
      
      sub_steering_ = nh.subscribe( "manual_control/steering", 1,  &steering_light_control::steeringCallback,this);
      sub_light_ = nh.subscribe( "manual_control/lights", 1,  &steering_light_control::lightCallback,this);
      pub=false;
      
    }
    ~steering_light_control(){}
    void steeringCallback(const std_msgs::Int16 steering_value);
    void lightCallback(const std_msgs::String light_value);
    bool pub;
    std::string light;
    int steering;
    send_steering_light steering_light;
    
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

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "send_steering_light_node");
  ros::NodeHandle nh;
  steering_light_control slc(nh);
  ros::Rate rate(50);
   while(ros::ok())
  {
    if (slc.pub==true)
    {
      slc.steering_light.run(slc.steering,slc.light);
      slc.pub=false;
    }
    ros::spinOnce();  
    rate.sleep();
  }
  return 0;
}
