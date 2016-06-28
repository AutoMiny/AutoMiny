#include <motor_communication/motor_communication.h>
#include <geometry_msgs/Twist.h>

class motor_control
{
  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_speed_;
    ros::Subscriber sub_steering_;
    ros::Subscriber sub_stop_;
    ros::Publisher pub_velocity_;

  public:
    motor_control(ros::NodeHandle nh) : nh_(nh)
    {
      sub_speed_ = nh_.subscribe( "motor_control/speed", 1, &motor_control::motorSpeedCallback,this);
      sub_stop_ = nh_.subscribe( "motor_control/stop_start", 1,  &motor_control::motorStopStartCallback,this);
      pub_velocity_=nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("motor_control/twist"), 1);
    }
    ~motor_control(){}
    void motorSpeedCallback(const std_msgs::Int16 speed_value);
    void motorStopStartCallback(const std_msgs::Int16 stop_value);
    void publishMotorTwist();
    motor_communication motor;
};

void motor_control::motorSpeedCallback(const std_msgs::Int16 speed_value)
{
  int speed;
  speed = speed_value.data;
  motor.run(speed);
}

void motor_control::motorStopStartCallback(const std_msgs::Int16 stop_value)
{ 
  if (stop_value.data==1)
  { 
    motor.stop();
  }
  else
  {
    motor.start();
  }
  
}
void motor_control::publishMotorTwist()
{ 
  double currentSpeed = motor.getSpeed(); 
  geometry_msgs::Twist currentTwist;
  currentTwist.linear.x = currentSpeed;
  currentTwist.linear.y = 0.0;
  currentTwist.linear.z = 0.0;
  pub_velocity_.publish(currentTwist);
  
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "motor_communication_node");
  ros::NodeHandle nh;
  motor_control MC1(nh);
  ros::Rate loop_rate(30);
   while(ros::ok())
  {
    MC1.publishMotorTwist();
    ros::spinOnce();
    loop_rate.sleep();
  }
  std_msgs::Int16 stop_value;
  stop_value.data=1;
  MC1.motorStopStartCallback(stop_value);
  return 0;
}
