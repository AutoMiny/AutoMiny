#!/usr/bin/env python3
import math

import numpy as np
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
import h5py
from tf.transformations import euler_from_quaternion
import tf

class VectorfieldController:
    def __init__(self):
        rospy.init_node('VectorfieldController')
        self.speed_value = 0.3
        self.last_angle = 0.0
        self.Kp = 4.0
        self.Kd = 0.2
        self.Ki = 0.0
        self.last_time = rospy.Time.now()
        self.integral_error = 0.0
        self.listener = tf.TransformListener()
        self.lane = None
        self.vector_field = h5py.File("vectorfield.hdf5", "r")

        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand,
                                         queue_size=1, tcp_nodelay=True)
        rospy.on_shutdown(self.shutdown)

        self.shutdown_ = False
        self.pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand,
                                   queue_size=1, tcp_nodelay=True)
        self.sub_odom = rospy.Subscriber("/simulation/odom_ground_truth", Odometry, self.callback, queue_size=1)
        self.lane_sub = rospy.Subscriber("/navigation/lane", Int64, self.on_lane, queue_size=1)

    def get_vector(self, odom, lane_id):
        position: Odometry
        vectors = self.vector_field["{}/forces".format(lane_id)]

        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        precision = vectors.attrs["precision"]
        bbox = vectors.attrs["bbox"]

        # clip to ranges of the bounding box for this specific vector field
        x = max(x, bbox[0][0])
        x = min(x, bbox[1][0])
        y = max(y, bbox[0][1])
        y = min(y, bbox[1][1])
        x -= bbox[0][0]
        y -= bbox[0][1]

        x_index = round(x / precision)
        y_index = round(y / precision)

        return vectors[x_index, y_index]

    def on_lane(self, lane):
        self.lane = lane.data

    def callback(self, odom):
        if self.lane is None:
            return

        dt = (odom.header.stamp - self.last_time).to_sec()
        self.last_time = odom.header.stamp

        orientation_q = odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        u, v = self.get_vector(odom, self.lane)
        f_x = np.cos(yaw) * u + np.sin(yaw) * v
        f_y = -np.sin(yaw) * u + np.cos(yaw) * v

        angle = np.arctan2(f_y, f_x)

        self.integral_error = self.integral_error + angle * dt
        steering = self.Kp * angle + self.Kd * ((angle - self.last_angle) / dt) + self.Ki * self.integral_error
        self.last_angle = angle

        speed = self.speed_value * np.sqrt(u**2 + v**2)
        steering = np.clip(steering, -1, 1)
        print(angle)

        print(speed)
        steerMsg = NormalizedSteeringCommand()
        steerMsg.value = steering
        steerMsg.header.frame_id = "base_link"
        steerMsg.header.stamp = rospy.Time.now()
        self.pub.publish(steerMsg)

        if not self.shutdown_:
            msg = SpeedCommand()
            msg.value = speed
            msg.header.frame_id = "base_link"
            msg.header.stamp = rospy.Time.now()
            self.pub_speed.publish(msg)

    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
        msg = SpeedCommand()
        msg.value = 0
        msg.header.frame_id = "base_link"
        self.pub_speed.publish(msg)
        rospy.sleep(1)

def main():
    try:
        VectorfieldController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("VectorfieldController node terminated.")


if __name__ == '__main__':
    main()
