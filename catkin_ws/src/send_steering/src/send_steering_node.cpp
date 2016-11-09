#include <send_steering.h>

class manual_control
{
  private:
    ros::Subscriber sub_steering_;
    send_steering steering;

  public:
    manual_control(ros::NodeHandle nh)
    {
      
      sub_steering_ = nh.subscribe( "manual_control/steering", 1,  &manual_control::manualSteeringCallback,this);
      
    }
    ~manual_control(){}
    void manualSteeringCallback(const std_msgs::Int16 steering_value);
    
};

void manual_control::manualSteeringCallback(const std_msgs::Int16 steering_value)
{ 
  steering.run(steering_value.data);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "send_steering_node");
  ros::NodeHandle nh;
  manual_control mc1(nh);
   while(ros::ok())
  {
    ros::spinOnce();  
  }
  return 0;
}