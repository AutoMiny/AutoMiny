#include <lights.h>

class manual_control
{
  private:
    ros::Subscriber sub_steering_;
    lights lights_;

  public:
    manual_control(ros::NodeHandle nh)
    {
      
      sub_steering_ = nh.subscribe( "manual_control/lights", 1,  &manual_control::manualCallback,this);
      
    }
    ~manual_control(){}
    void manualCallback(const std_msgs::String light_value);
    
};

void manual_control::manualCallback(const std_msgs::String light_value)
{ 
  //lights_.run("enL");

  lights_.run(light_value.data);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "lights_node");
  ros::NodeHandle nh;
  manual_control light(nh);
   while(ros::ok())
  {
    ros::spinOnce();  
  }
  std_msgs::String stop_value;
  stop_value.data="diL";
  light.manualCallback(stop_value);
  return 0;
}