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
import math
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import tf

traj_publisher = rospy.Publisher("/model_car/trajectory", Trajectory, queue_size =10)
sequence =0
listener = tf.TransformListener()

def conv_to_traj(data):
    global sequence
    new_traj = Trajectory()
    new_traj.header.seq = sequence
    sequence += 1 #increment counter
    new_traj.header.stamp = rospy.Time.now()
    new_traj.header.frame_id = "/odom" #TODO change to /map frame
    new_traj.child_frame_id = "/base_link"
    no_of_poses = len(data.poses)
    time_ = rospy.Time.now()
    print "no_of_poses : ",no_of_poses
    for i in range(no_of_poses):
        pt_Stamped_in = PointStamped()
        pt_Stamped_in.header.seq =1
        pt_Stamped_in.header.stamp =time_
        pt_Stamped_in.header.frame_id= "/map"
        pt_Stamped_in.point.x = data.poses[i].pose.position.x
        pt_Stamped_in.point.y = data.poses[i].pose.position.y
        pt_Stamped_in.point.z = 0;
        #print "x,y ", data.poses[i].pose.position.x, data.poses[i].pose.position.y
        tpt = listener.transformPoint("/odom",pt_Stamped_in)
        tp = TrajectoryPoint()
        #pose-position
        tp.pose.position.x = tpt.point.x
        tp.pose.position.y = tpt.point.y
        #print "xy :: ",tp.pose.position.x,"  ",tp.pose.position.y
        tp.pose.position.z = 0.00
        if(i < (no_of_poses-1)):
            #slope = ((points[i+1][1]-points[i][1])/(points[i+1][0] - points[i][0]))
            #print "il::",i,"slope::",slope
            yaw = math.atan2((data.poses[i+1].pose.position.y - data.poses[i].pose.position.y),(data.poses[i+1].pose.position.x-data.poses[i].pose.position.x))
        else:
            #slope = ((points[i][1]-points[i-1][1])/(points[i][0] - points[i-1][0]))
            #print "idk::",i,"slope::",slope
            yaw = math.atan2((data.poses[i].pose.position.y - data.poses[i-1].pose.position.y),(data.poses[i].pose.position.x-data.poses[i-1].pose.position.x))

        # RPY to convert: 0deg, 0, slope of line as rotation- yaw
        #sxyz - roll, pitch, yaw
        yaw = yaw*180.0/math.pi
        #print "yaw degrees ::",yaw
        qat = quaternion_from_euler(0, 0, yaw,axes='sxyz')
        #print "qat ",qat[0]," ",qat[1]," ",qat[2]," ",qat[3]
        tp.pose.orientation.x = qat[0]
        tp.pose.orientation.y = qat[1]
        tp.pose.orientation.z = qat[2]
        tp.pose.orientation.w = qat[3]

        tp.velocity.linear.x = data.poses[i].pose.position.z
        tp.acceleration.linear.x = 0#data.poses[i].pose.orientation.x
        #25 points are published with 0.2s of time gap
        tp.time_from_start =rospy.Duration(0.2*i)

        #Append this point to the array of traj points
        new_traj.trajectory.append(tp)

    traj_publisher.publish(new_traj)

    ##End of function




def main(args):
    rospy.init_node('path_to_fub_traj_converter', anonymous = False)
    rospy.Subscriber("/motionplanner/traj_1", Path, conv_to_traj)

    rospy.spin()










if __name__ == '__main__':
    main(sys.argv)
