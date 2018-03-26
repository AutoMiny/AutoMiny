#!/usr/bin/env python
import time
import roslib
import rospy
import sys
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Pose, Point, Twist, Quaternion,Vector3, PoseWithCovarianceStamped, PoseStamped
import time
from fub_trajectory_msgs.msg import Trajectory
from fub_trajectory_msgs.msg import TrajectoryPoint
import math
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import tf
from nav_msgs.msg import Odometry

odom_pub = rospy.Publisher("/odom", Odometry, queue_size =10)
rcv_pose = PoseStamped()
listener = tf.TransformListener()

def pub_odom():
    vel =0.3
    #print rcv_pose

    if rcv_pose.pose.orientation.w == 0:
        x = 0.0#4.19#4.5
        y = 0#-2.48#0.0
        current_time = rospy.Time.now()
        odom_broadcaster = tf.TransformBroadcaster()
        #since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0,0,0.00)
        #first, we'll publish the transform over tf
        odom_broadcaster.sendTransform((0, 0, 0),odom_quat,current_time,"base_link","odom")
        #next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vel, 0, 0), Vector3(0, 0, 0))
        #publish the message
        odom_pub.publish(odom)
    else:
        current_time = rospy.Time.now()
        odom_broadcaster = tf.TransformBroadcaster()
        try:
            pose_odom = listener.transformPose("/odom",rcv_pose)
            #since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = (pose_odom.pose.orientation.x,pose_odom.pose.orientation.y, pose_odom.pose.orientation.z, pose_odom.pose.orientation.w)
            #first, we'll publish the transform over tf
            odom_broadcaster.sendTransform((0, 0, 0),odom_quat,current_time,"base_link","odom")
            #next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.pose.pose = pose_odom.pose
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vel, 0, 0), Vector3(0, 0, 0))
            #publish the message
            odom_pub.publish(odom)
        except Exception as e:
            pass
            #print e

def odometry_from_rviz(data):
    global rcv_pose
    rcv_pose.header = data.header
    rcv_pose.pose = data.pose.pose


def main(args):
    rospy.init_node('odom_pub', anonymous = False)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, odometry_from_rviz)
    rate = rospy.Rate(3) # 10hz
    while not rospy.is_shutdown():
        pub_odom()
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
