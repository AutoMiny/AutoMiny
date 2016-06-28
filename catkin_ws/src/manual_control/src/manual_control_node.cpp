#include <send_steering.h>

class manual_control
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_steering_;

  public:
    manual_control(ros::NodeHandle nh) : nh_(nh)
    {
      sub_steering_ = nh_.subscribe( "manual_control/steering", 1,  &manual_control::manualSteeringCallback,this);
    }
    ~manual_control(){}
    void manualSteeringCallback(const std_msgs::Int16 steering_value);
    send_steering steering;
};

void manual_control::manualSteeringCallback(const std_msgs::Int16 steering_value)
{ 
  int a;
  a = steering_value.data;
  steering.run(a);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "manual_control_node");
  ros::NodeHandle nh;
  manual_control MC1(nh);
   while(ros::ok())
  {
    ros::spinOnce();  
  }
  return 0;
}
