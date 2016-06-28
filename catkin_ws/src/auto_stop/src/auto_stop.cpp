#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

//parameters
#include <ros/ros.h>

class auto_stop
{
public:
	auto_stop(ros::NodeHandle nh)
	{
		n_.param<int>("angle_front", angle_front, 40);
		n_.param<int>("angle_back", angle_back, 40);
		n_.param<float>("break_distance", break_distance, 0.45);
		pubEmergencyStop_=nh.advertise<std_msgs::Int16>(nh.resolveName("manual_control/speed"), 1);
		subScan_ = n_.subscribe("scan", 1, &auto_stop::scanCallback,this);
		subTwist_ = n_.subscribe("motor_control/twist",1,&auto_stop::speedCallback,this); 
	}
	~auto_stop(){}

    void speedCallback(const geometry_msgs::Twist& twist)
	{
		direction=twist.linear.x;
	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
	{
	    int count = scan->scan_time / scan->time_increment;
	    float  break_distance_=break_distance;
	    if (abs(direction)>500)
	    	break_distance_=(abs(direction)/500)*break_distance;
	    std_msgs::Int16 speed;
	    speed.data=0;
	    //ROS_INFO("speed %f",break_distance_);	
		if(direction < 0){	//backw.
			for(int i = 0; i < (angle_back/2)+1; i++){
				if (scan->ranges[i] <= break_distance_){
				
					pubEmergencyStop_.publish(speed);
					//ROS_INFO("Obstacle");
					return;
			    }
			}
			for(int k = (360-(angle_back/2)); k < count; k++){
				if (scan->ranges[k] <= break_distance_){
					pubEmergencyStop_.publish(speed);
					return;
			    }
			}
		}

		if(direction > 0){ //forw.
			for(int j = (180-(angle_front/2)); j < (180+(angle_front/2))+1; j++){
				if (scan->ranges[j] <= break_distance_){
					pubEmergencyStop_.publish(speed);
					return;
			    }
			}
		}
	}

	private:
	  	int angle_front;
		int angle_back;
		float break_distance;
		int direction;
	  	ros::NodeHandle n_; 
	  	ros::Publisher pubEmergencyStop_;
	  	ros::Subscriber subScan_;
	  	ros::Subscriber subTwist_;

};//End of class auto_stop



int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_stop_node");
    ros::NodeHandle nh; 
    auto_stop autoStopObject(nh);

	while(ros::ok())
	{
		ros::spin();
	}
    return 0;
}