#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

class auto_stop
{
public:
	auto_stop(ros::NodeHandle nh, ros::NodeHandle nh_)
	{
		nh_.param<int>("angle_front", angle_front, 40);
		nh_.param<int>("angle_back", angle_back, 40);
		nh_.param<float>("break_distance", break_distance, 0.45);
		nh_.param<bool>("break_distance_based_on_speed", break_distance_based_on_speed, false);

		ROS_INFO_STREAM("break_distance "<< break_distance << " m, angle_front " << angle_front <<" degrees, angle_back " << angle_back <<" degrees");
		speedCommand.data=0;
		emergencyStop.data=0;
    direction = 0;
		pubSpeed_ = nh.advertise<std_msgs::Int16>(nh.resolveName("/checked/speed"), 1);
		subScan_ = nh_.subscribe("/scan", 1, &auto_stop::scanCallback,this);
		subTwist_ = nh_.subscribe("/motor_control/twist",1,&auto_stop::speedCallback,this);
		subSpeedCommand_= nh_.subscribe("/manual_control/speed",1,&auto_stop::speedCommandCallback,this);
	}
	~auto_stop(){}

	void speedCommandCallback(const std_msgs::Int16& input)
	{
		ROS_INFO_STREAM("speed command " << input.data);
		speedCommand=input;
	}
    void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x; 
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    int count = scan->scan_time / scan->time_increment;
	    float break_distance_=break_distance;
	    if (abs(direction)>50 & (break_distance_based_on_speed==true))
	    	break_distance_=(abs(direction)/50)*break_distance;
	    //ROS_INFO("speed %f",break_distance_);
		if(speedCommand.data < 0){	//backw.
			for(int i = 0; i < (angle_back/2)+1; i++){
				if (scan->ranges[i] <= break_distance_){
					pubSpeed_.publish(emergencyStop);
					ROS_INFO("Emergency Stop");
					return;
			    }
			}
			for(int k = (360-(angle_back/2)); k < count; k++){
				if (scan->ranges[k] <= break_distance_){
					pubSpeed_.publish(emergencyStop);
					ROS_INFO("Emergency Stop");
					return;
			    }
			}
		}

		if(speedCommand.data > 0){ //forw.
			for(int j = (180-(angle_front/2)); j < (180+(angle_front/2))+1; j++){
				if (scan->ranges[j] <= break_distance_){
					pubSpeed_.publish(emergencyStop);
					ROS_INFO("Emergency Stop");
					return;
			    }
			}
		}
		//There is not any obstacle
		pubSpeed_.publish(speedCommand);
		return;

	}

	private:
	  	int angle_front;
		int angle_back;
		float break_distance;
		int direction;
		bool break_distance_based_on_speed;
		std_msgs::Int16 speedCommand;
		std_msgs::Int16 emergencyStop;
	  	ros::Publisher  pubSpeed_;
	  	ros::Subscriber subScan_;
	  	ros::Subscriber subTwist_;
	  	ros::Subscriber subSpeedCommand_;

};//End of class auto_stop



int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_stop_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");
    auto_stop autoStopObject(nh, nh_);

	while(ros::ok())
	{
		ros::spin();
	}
    return 0;
}
