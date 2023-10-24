#!/usr/bin/python3

import rclpy
import math
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def euler_from_quaternion(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class GPSInitialpose(Node):

    def __init__(self):
        super().__init__("gps_initialpose")
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.gps_sub = self.create_subscription(Odometry, "/sensors/odometry/gps", self.on_gps, 1)
        self.localization_sub = self.create_subscription(Odometry, "/sensors/localization/filtered_map", self.on_localization, 1)
        self.localization = None
        self.declare_parameter("tolerance_xy", 0.4)
        self.declare_parameter("tolerance_yaw", 1.5)
        self.tolerance_xy = self.get_parameter("tolerance_xy").value
        self.tolerance_yaw = self.get_parameter("tolerance_yaw").value
        self.last_correction = self.get_clock().now()

    def on_localization(self, msg):
        self.localization = msg

    def on_gps(self, msg):
        if self.localization and (self.get_clock().now() - self.last_correction).nanoseconds / 1e9 > 3.0:
            distance = math.sqrt((self.localization.pose.pose.position.x - msg.pose.pose.position.x) ** 2.0 + (
                    self.localization.pose.pose.position.y - msg.pose.pose.position.y) ** 2.0)
            yaw_localization = euler_from_quaternion(self.localization.pose.pose.orientation.x,
                                                     self.localization.pose.pose.orientation.y,
                                                     self.localization.pose.pose.orientation.z,
                                                     self.localization.pose.pose.orientation.w)[2]
            yaw_gps = euler_from_quaternion(
                msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)[2]
            delta_yaw = math.fabs(yaw_localization - yaw_gps)

            if distance > self.tolerance_xy:  # or delta_yaw > self.tolerance_yaw:
                print (distance, delta_yaw)
                pose = PoseWithCovarianceStamped()
                pose.pose = msg.pose
                pose.header.frame_id = "map"
                pose.header.stamp = self.get_clock().now().to_msg()
                self.initialpose_pub.publish(pose)
                self.last_correction = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)

    node = GPSInitialpose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
