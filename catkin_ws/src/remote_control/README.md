# Remote Control

## Dependencies

	apt-get install libwebsockets-dev libboost-all-dev


## How to build

	$ source /opt/ros/<ros-distribution>/setup.bash
	$ cd <path-to-catkin_ws>
	$ catkin_make remote_control_node

## How to run

	$ cd <path-to-catkin_ws>
	$ source devel/setup.bash
	$ rosrun remote_control remote_control_node

If you want to use a remote roscore:

	$ export ROS_MASTER_URI=http://<master-ip>:<master-port>/
	$ export ROS_IP=<own-ip>

## Contribution Workflow

	$ Create one branch per feature.
	$ Open a merge request.
	$ Discuss and modify.
	$ Assignee merges if everything is fine.

## Class Hierarchy
 
![alt text][fig_structure]

[fig_structure]: https://github.com/schra/fub_modelcar/blob/master/model_car/catkin_ws/src/remote_control/doc/architecture.png "class diagram"