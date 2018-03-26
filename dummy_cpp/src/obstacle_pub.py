#!/usr/bin/env python
import time
import roslib
import rospy
import sys
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import time
from fub_trajectory_msgs.msg import Trajectory
from fub_trajectory_msgs.msg import TrajectoryPoint
from autonomos_obstacle_msgs.msg import Obstacle
from autonomos_obstacle_msgs.msg import Obstacles
import math
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovariance
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped, Pose, Point, Twist, Quaternion,Vector3

obst_pub = rospy.Publisher("obstacles",Obstacles, queue_size = 5)
sequence =0
var_abs_vel = 0

def pub_obstacle_info():
    global sequence,var_abs_vel
    current_time = rospy.Time.now()
    new_obstacles_list = Obstacles()
    new_obstacles_list.header.stamp = current_time
    new_obstacles_list.header.frame_id = "map"
    new_obstacles_list.header.seq = sequence
    sequence +=1
    obs_vel = [0.25,0.0,0.0]
    obs_xy = [[2.3,-0.19],[3.35,2.3],[3.69,2.4]]
    for id_ in range(3):
        new_obstacle = Obstacle()
        new_obstacle.id = id_
        new_obstacle.header.stamp = current_time
        new_obstacle.header.frame_id = "map"
        new_obstacle.header.seq = id_
        var_abs_vel = obs_vel[id_]
        x = obs_xy[id_][0]#0.3*id_ +2.0 #id_*2.0#4.19#4.5
        y = obs_xy[id_][1]#-2.48#0.0
        #Obstacle odometry
        odom_quat = tf.transformations.quaternion_from_euler(0,0,0)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "map"
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(var_abs_vel, 0, 0), Vector3(0, 0, 0))
        new_obstacle.odom = odom
        #TwistWithCovariance
        abs_vel = TwistWithCovariance()
        abs_vel.twist.linear.x = var_abs_vel
        abs_vel.twist.linear.y = 0.0
        new_obstacle.abs_velocity = abs_vel

        #reference_point - stable point on obstacle
        ref_pt = Point32()
        ref_pt.x = x
        ref_pt.y = y
        ref_pt.z = 0
        new_obstacle.reference_point = ref_pt
        #Bounding box min, max wrt to center - given by odom pose
        pt_min = Point32()
        pt_min.x = -0.05
        pt_min.y = -0.03
        pt_min.z = 0
        pt_max = Point32()
        pt_max.x = 0.05
        pt_max.y = 0.03
        pt_max.z = 0.1
        new_obstacle.bounding_box_min = pt_min
        new_obstacle.bounding_box_max = pt_max

        new_obstacle.classification = 1
        new_obstacle.classification_certainty = 0.9
        new_obstacle.first_observed = current_time #- rospy.Duration(20)
        new_obstacle.last_observed = current_time

        new_obstacles_list.obstacles.append(new_obstacle)
    obst_pub.publish(new_obstacles_list)



def main():
    rospy.init_node("obstacle_publisher", anonymous = False)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        pub_obstacle_info()
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
