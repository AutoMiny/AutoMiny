#!/usr/bin/env python
import random
import time
import roslib
import rospy
import sys
import numpy as np
#from matplotlib import pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
#from matplotlib import animation
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import tf

#file2write=open("odom_car_2.txt",'w')

speed = 0
steer = 0
once = 0
inti_time = None

def speed_callback(data):
    global speed,inti_time,once
    speed = data.data
    print "speed ",speed
    once = 0
    inti_time = rospy.Time.now()

def steer_callback(data):
    global steer
    steer = data.data
    print "steer ",steer

"""
def odom_callback(data):
    global x,y,z
    file2write.write(str(data.header.stamp.secs))
    file2write.write(",")
    file2write.write(str(data.pose.pose.position.x))
    file2write.write(",")
    file2write.write(str(data.pose.pose.position.y))
    file2write.write(",")
    file2write.write(str(data.twist.twist.linear.x))
    file2write.write(",")
    file2write.write(str(speed))
    file2write.write(",")
    file2write.write(str(steer))
    file2write.write("\n")
"""
def yaw_callback(data):
    pass
    #rospy.loginfo("raw_yaw %f",data.data)



def odom_callback(data):
    global once,inti_time
    #print rospy.Time.now().secs,".",rospy.Time.now().nsecs
    orien = [data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    _,_,yaw = tf.transformations.euler_from_quaternion(orien)
    #print "x,y : ",data.pose.pose.position.x," , ",data.pose.pose.position.y," vel : ",data.twist.twist.linear.x," yaw ",yaw
    if(once==0 and data.twist.twist.linear.x>0.05):
	print"##############################################"
	print "#######Delay ",(rospy.Time.now() - inti_time).secs,",",(rospy.Time.now() - inti_time).nsecs
	print "latency  ",(data.header.stamp - inti_time)
	once = 1 

def main(args):
    global once,inti_time
    rospy.init_node('odom_plotter', anonymous = False)
    #print('Publish odometry from car/rosbag to record to odom_car.txt ')
    i_sub = rospy.Subscriber("/odom",Odometry, odom_callback)
    i_sub1 = rospy.Subscriber("/manual_control/speed",Int16, speed_callback)
    i_sub2 = rospy.Subscriber("/manual_control/steering",Int16, steer_callback)
    yaw_s  = rospy.Subscriber("/model_car/yaw",Float32,yaw_callback)
    inti_time = rospy.Time.now()   
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
