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

### Fisheye-Camera settings
#v4l2-ctl --device=/dev/usb_cam --set-ctrl exposure_auto=1
#v4l2-ctl --device=/dev/usb_cam --set-ctrl exposure_absolute=7

### Start ROS launch scripts
roslaunch manual_control manual_odroid.launch &
roslaunch map_publisher robotics_lab.launch &
sleep 10
roslaunch odometry odometry.launch 
#roslaunch random_movement auto.launch

### Start Web Control Center
#/root/web-control-center/start-server.sh &
