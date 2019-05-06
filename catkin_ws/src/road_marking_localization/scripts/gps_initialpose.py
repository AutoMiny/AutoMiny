#! /usr/bin/python

import rospy
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class GPSInitialpose:

    def __init__(self):
        self.initialpose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=1)
        self.gps_sub = rospy.Subscriber("gps", Odometry, self.on_gps, queue_size=1)
        self.localization_sub = rospy.Subscriber("localization", Odometry, self.on_localization, queue_size=1)
        self.localization = None
        self.tolerance_xy = rospy.get_param("tolerance_xy", 0.4)
        self.tolerance_yaw = rospy.get_param("tolerance_yaw", 1.5)
        self.last_correction = rospy.Time.now()

    def on_localization(self, msg):
        self.localization = msg

    def on_gps(self, msg):
        if self.localization and (rospy.Time.now() - self.last_correction).to_sec() > 3.0:
            distance = math.sqrt((self.localization.pose.pose.position.x - msg.pose.pose.position.x) ** 2.0 + (
                    self.localization.pose.pose.position.y - msg.pose.pose.position.y) ** 2.0)
            yaw_localization = euler_from_quaternion((self.localization.pose.pose.orientation.x,
                                                      self.localization.pose.pose.orientation.y,
                                                      self.localization.pose.pose.orientation.z,
                                                      self.localization.pose.pose.orientation.w))[2]
            yaw_gps = euler_from_quaternion(
                (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w))[2]
            delta_yaw = math.fabs(yaw_localization - yaw_gps)

            if distance > self.tolerance_xy:  # or delta_yaw > self.tolerance_yaw:
                print (distance, delta_yaw)
                pose = PoseWithCovarianceStamped()
                pose.pose = msg.pose
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                self.initialpose_pub.publish(pose)
                self.last_correction = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('gps_initialpose')
    GPSInitialpose()
    rospy.spin()
