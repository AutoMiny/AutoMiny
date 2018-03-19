#!/usr/bin/env python
# coding: utf-8

# imports
import sys
import roslib
import rospy
import numpy as np
import pickle

# Message types
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan


# we do somehing similar
def drive_angle(angle):
    STRAIGHT = 105 # measured with car
    SPEED = 100 #decent speed
    DURATION = 10

    # setup the topics
    speed = rospy.Publisher('/manual_control/speed', Int16, queue_size=10)
    steer = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
    chatter = rospy.Publisher('chatter', String, queue_size=10)

    # get a first measurement:
    data = rospy.wait_for_message("scan", LaserScan)
    # m1 = evaluate_lidar(data)
    filename = "steering_"+angle+"_01.pkl"
    with open(filename, 'wb') as output:
        pickle.dump(data, output, pickle.HIGHEST_PROTOCOL)
    rospy.sleep(1)
    
    # set steering and drive back   
    #steer.publish(0)
    steer.publish(int(angle))
    rospy.sleep(1)
    speed.publish(SPEED)
    rospy.sleep(DURATION)
    speed.publish(0)
    rospy.sleep(1)
    

    # get the second measurement:
    data = rospy.wait_for_message("scan", LaserScan)
    # m2 = evaluate_lidar(data)
    filename = "steering_"+angle+"_02.pkl"
    with open(filename, 'wb') as output:
        pickle.dump(data, output, pickle.HIGHEST_PROTOCOL)
    
    steer.publish(STRAIGHT)

    # Assemble message and send to our listener
    message = "data of " + angle + " is saved"
    print(message)

    # stop  
    exit()  

def evaluate_lidar(data):

    # convert to x,y
    off = data.angle_min
    inc = data.angle_increment
    l = len(data.ranges)

    # X: Liste von Listen, y Liste voller Nullen
    X = np.ones((l,2))
    y = np.ones((l))

    # assuming data comes in 360 degs
    for i in range(l):
        a = (off + i*inc)
        v = np.array([np.cos(a), np.sin(a)])  # vgl. Kreisformel
        p = data.ranges[i] * v
        X[i,0] = p[0]
        y[i] = p[1]

    # fit line:
    print('HERE')
    a, b = np.linalg.lstsq(X, y, rcond=-1)
    print('HERE 2')

    # get d
    if a == 0:
        alpha = np.pi 
        d = b
    elif a > 0:
        alpha = np.arctan(a)
        d = np.sin(aplha) * b
    else:
        alpha = np.arctan(a) + np.pi
        d = np.sin(aplha) * b

    return(a, b, d, alpha)

def main(args):
    rospy.init_node("drive_angle")
    if len(args) > 1:
        try:
            drive_angle(args[1])
        except rospy.ROSInterruptException:
            pass
    else:
        print("please provide a steering setting from [0,180]") 

if __name__ == '__main__':
    main(sys.argv)
