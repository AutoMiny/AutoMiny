modprobe uvcvideo
export ROS_HOSTNAME=192.168.43.102
export ROS_MASTER_URI=http://192.168.43.102:11311
source /opt/ros/indigo/setup.bash
cd catkin_ws
source devel/setup.bash
timeout 10 roslaunch realsense_camera realsense_sr300.launch 
#roslaunch manual_control manual_odroid.launch || exit
roslaunch random_movement auto.launch || exit


