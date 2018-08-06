#!/usr/bin/env python2

import rospy

from path_parser import read_points

from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
import math


def read_map(map_file):
    """
    Read the file with the map data and return a Trajectory.
    :return: List of Pose elements
    """
    points = []

    for x, y in read_points(map_file):
        xy_point = Point(x, y, 0)
        xy_pose  = Pose(position=xy_point)
        points.append(xy_pose)

    return points

path = read_map('new_map_loop1.txt')

start_time = 0
last_time = 0
score = 0
count = 0

def odomCallback(odom):
    global last_time, score, start_time, count
    distance = findNearestPointOnPath(odom)
    delta_t = odom.header.stamp - last_time;
    last_time = odom.header.stamp
    count += 1
    score += distance
    time_since_start = rospy.Time.now() - start_time
    print("error: {} time: {}".format(score / count, time_since_start.to_sec()))
    #scorePub.publish(score)

def euclidean(odom, pose):
    return math.sqrt((odom.pose.pose.position.x - pose.position.x)**2.0 + (odom.pose.pose.position.y - pose.position.y)**2.0)

def findNearestPointOnPath(odom):
    min = 1000
    for pose in path:
        if (euclidean(odom, pose)) < min:
            min = euclidean(odom, pose)
    return min
        
def main():
    rospy.init_node('assignment10_trajectory')
    global last_time, scorePub, start_time

    raw_input("Press Enter to continue...")
    last_time = rospy.Time.now()
    start_time = rospy.Time.now()
    odomSub = rospy.Subscriber("/localization/odom/3", Odometry, odomCallback, queue_size = 10)
    scorePub = rospy.Publisher("/score", Float64, queue_size = 10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()

