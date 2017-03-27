modprobe uvcvideo
export ROS_HOSTNAME=192.168.43.102
export ROS_MASTER_URI=http://192.168.43.102:11311
source /opt/ros/indigo/setup.bash
cd catkin_ws
source devel/setup.bash
roslaunch manual_control manual_odroid.launch || exit



