class auto_control
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle priv_nh_;
    ros::Subscriber sub_curvature_;
    ros::Publisher pub_steering_;
    ros::Publisher pub_speed_;


    float steering_Kp_;
    float steering_Kd_;
    float steering_Ki_;
    float speed_Kp_;
    float maximum_rpm_;
    float minimum_rpm_;
    float maximum_steering_;
    float minimum_steering_;
    float angle;
    float last_angle;

    std_msgs::Int16 desired_steering;
    std_msgs::Int16 desired_speed;

  public:
    anto_control(ros::NodeHandle nh) : nh_(nh), priv_nh_("~"),
    {
      priv_nh_.param<float>("steering_Kp", steering_Kp_, 0.7);
      priv_nh_.param<float>("steering_Kd", steering_Kd_, 1.0);
      priv_nh_.param<float>("steering_Ki", steering_Ki_, 0.0);
      priv_nh_.param<float>("speed_Kp", speed_Kp_, 0.05);
      priv_nh_.param<float>("maximum_rpm", maximum_rpm_, 1000);
      priv_nh_.param<float>("minimum_rpm", minimum_rpm_, 20);
      priv_nh_.param<float>("maximum_steering", maximum_steering_, 50);
      priv_nh_.param<float>("minimum_steering", minimum_steering_, -50);
      sub_steering_ = nh_.subscribe( "vision/curvature", 1,  &auto_control::curvatureCallback,this);
      pub_steering_= nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_contol/steering"), 1);
      pub_speed_= nh.advertise<std_msgs::Int16>(nh.resolveName("/manual_contol/speed"), 1);

    }
    ~auto_control(){}
    void curvatureCallback(const std_msgs::Float32 curvaturee);
};

void auto_control::curvatureCallback(const std_msgs::Float32 curvature)
{ 

  //angle based on curvature
  angle = (90.0 - (atan2(1,curvature.data) /(2 * 3.14) * 360.0));
  int DesiredSteering;
  int DesiredSpeed;

  DesiredSteering=steering_Kp_*angle+steering_Kd_*((angle-last_angle)/sampleTime);
  last_angle = angle;
  if (DesiredSteering>maximum_steering_)
    DesiredSteering=maximum_steering_;
  else if (DesiredSteering<minimum_steering_)
    DesiredSteering=minimum_steering_;

  
  if ((DesiredSteering<1)&&(-1<DesiredSteering))
    DesiredSpeed=maximum_rpm_;
  else
      DesiredSpeed=minimum_steering_+speed_Kp_*(maximum_steering_/abs(DesiredSteering));

  if  (DesiredSpeed>maximum_rpm_)
      DesiredSpeed=maximum_rpm_; 

  desired_steering.data=DesiredSteering;
  desired_speed.data=DesiredSpeed;
  pub_steering_.publish(desired_steering);
  pub_speed_.publish(desired_speed);
}

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "auto_control_node");
  ros::NodeHandle nh;
  auto_control control1(nh);
   while(ros::ok())
  {
    ros::spinOnce();  
  }
  return 0;
}