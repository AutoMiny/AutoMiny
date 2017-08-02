#!/bin/bash
modprobe uvcvideo

# clear ROS log
rm -rf /root/.ros/log

# load environment settings from .bashrc
PS1='$ '
source /root/.bashrc

### ROS Setup (is now loaded from .bashrc)
#export ROS_MASTER_URI=http://$IP:11311
#...

### Start ROS core and wait a few seconds
roscore &
sleep 5

### Start ROS launch scripts
roslaunch manual_control manual_odroid.launch &
#roslaunch random_movement auto.launch

### Start Web Control Center
#/root/web-control-center/start-server.sh &
