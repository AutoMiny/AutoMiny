#!/usr/bin/env python2
import math
import numpy as np
import rospkg
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, NormalizedSpeedCommand
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import tf

class VectorfieldController:
    def __init__(self):
        rospy.init_node('VectorfieldController')
        self.map_size_x = 600  # cm
        self.map_size_y = 430  # cm
        self.resolution = 1  # cm
        self.lane = 2
        self.last_lane_change = rospy.Time.now()
        self.speed_value = 0.3
        self.last_angle = 0.0
        self.Kp = 4.0
        self.Kd = 0.2
        self.Ki = 0.0
        self.last_time = rospy.Time.now()
        self.integral_error = 0.0
        self.listener = tf.TransformListener()

        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path('fub_navigation') + '/src/'
        self.matrix_lane_1 = np.load(self.file_path + 'matrix50cm_lane1.npy')
        self.matrix_lane_2 = np.load(self.file_path + 'matrix50cm_lane2.npy')
        self.distance_lane_1 = np.load(self.file_path + 'matrix0cm_lane1.npy')
        self.distance_lane_2 = np.load(self.file_path + 'matrix0cm_lane2.npy')


        print("speed", self.speed_value)
        if self.lane == 1:
            self.matrix = self.matrix_lane_1
        else:
            self.matrix = self.matrix_lane_2

        self.pub_speed = rospy.Publisher("/actuators/speed_normalized", NormalizedSpeedCommand,
                                         queue_size=1, tcp_nodelay=True)
        rospy.on_shutdown(self.shutdown)

        self.shutdown_ = False
        self.pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand,
                                   queue_size=1, tcp_nodelay=True)
        self.sub_odom = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.callback, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/sensors/rplidar/scan", LaserScan, self.on_lidar, queue_size=1)

    def callback(self, data):
        dt = (data.header.stamp - self.last_time).to_sec()
        # 25hz
        if dt < 0.04:
            return

        self.last_time = data.header.stamp
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        x_index_floor = int(math.floor(x * (100.0 / self.resolution)))
        y_index_floor = int(math.floor(y * (100.0 / self.resolution)))

        x_index_ceil = x_index_floor + 1
        y_index_ceil = y_index_floor + 1

        ceil_ratio_x = x * (100.0 / self.resolution) - x_index_floor
        ceil_ratio_y = y * (100.0 / self.resolution) - y_index_floor

        if x_index_floor < 0:
            x_index_floor = 0
        if x_index_floor > self.map_size_x / self.resolution - 1:
            x_index_floor = self.map_size_x / self.resolution - 1

        if y_index_floor < 0:
            y_index_floor = 0
        if y_index_floor > self.map_size_y / self.resolution - 1:
            y_index_floor = self.map_size_y / self.resolution - 1

        if x_index_ceil < 0:
            x_index_ceil = 0
        if x_index_ceil > self.map_size_x / self.resolution - 1:
            x_index_ceil = self.map_size_x / self.resolution - 1

        if y_index_ceil < 0:
            y_index_ceil = 0
        if y_index_ceil > self.map_size_y / self.resolution - 1:
            y_index_ceil = self.map_size_y / self.resolution - 1

        x3_floor, y3_floor = self.matrix[x_index_floor, y_index_floor, :]
        x3_ceil, y3_ceil = self.matrix[x_index_ceil, y_index_ceil, :]
        x3 = x3_floor * (1.0 - ceil_ratio_x) + x3_ceil * ceil_ratio_x
        y3 = y3_floor * (1.0 - ceil_ratio_y) + y3_ceil * ceil_ratio_y
        f_x = np.cos(yaw) * x3 + np.sin(yaw) * y3
        f_y = -np.sin(yaw) * x3 + np.cos(yaw) * y3

        angle = np.arctan2(f_y, f_x)

        self.integral_error = self.integral_error + angle * dt
        steering = self.Kp * angle + self.Kd * ((angle - self.last_angle) / dt) + self.Ki * self.integral_error
        self.last_angle = angle

        if f_x > 0:
            speed = -self.speed_value
        else:
            speed = self.speed_value

        if steering > 1:
            steering = 1

        if steering < -1:
            steering = -1
        if f_x > 0:
            speed = max(self.speed_value, speed * ((np.pi / 3) / (abs(steering) + 1)))

        steerMsg = NormalizedSteeringCommand()
        steerMsg.value = steering
        steerMsg.header.stamp = rospy.Time.now()
        self.pub.publish(steerMsg)

        if not self.shutdown_:
            msg = NormalizedSpeedCommand()
            msg.value = speed
            msg.header.stamp = rospy.Time.now()
            self.pub_speed.publish(msg)

    def shutdown(self):
        print("shutdown!")
        self.shutdown_ = True
        msg = NormalizedSpeedCommand()
        msg.value = 0
        self.pub_speed.publish(msg)
        rospy.sleep(1)

    def lane_change(self):
        if (rospy.Time.now() - self.last_lane_change).to_sec() < 1.0:
            return

        if self.lane == 1:
            self.matrix = self.matrix_lane_2
            self.lane = 2
        else:
            self.matrix = self.matrix_lane_1
            self.lane = 1
        self.last_lane_change = rospy.Time.now()
        print("lane change")

    def on_lidar(self, msg):
        points = []
        for i in range(len(msg.ranges) - 1, int(len(msg.ranges) * 0.85), -1):
            dist = msg.ranges[i]
            angle = i * msg.angle_increment
            if dist < 1.3:
                points.append((-dist * np.cos(angle), -dist * np.sin(angle)))

        for i in range(0, int(len(msg.ranges) * 0.15), 1):
            dist = msg.ranges[i]
            angle = i * msg.angle_increment
            if dist < 1.3:
                points.append((-dist * np.cos(angle), -dist * np.sin(angle)))

        points_on_track = 0

        if len(points) > 0:
            (t,r) = self.listener.lookupTransform("map", msg.header.frame_id, rospy.Time(0))
            mat44 = np.dot(tf.transformations.translation_matrix(t), tf.transformations.quaternion_matrix(r))

            for (x, y) in points:
                (xm, ym, zm) = tuple(np.dot(mat44, np.array([x, y, 0, 1.0])))[:3]
                (xi, yi) = int(xm * (100 / self.resolution)), int(ym * (100 / self.resolution))

                if 0 <= xi < 600 and 0 <= yi < 430:
                    if self.lane == 1:
                        (xd, yd) = self.distance_lane_1[xi, yi, :]
                    else:
                        (xd, yd) = self.distance_lane_2[xi, yi, :]
                    dist = np.sqrt(xd ** 2.0 + yd ** 2.0)
                    if dist < 0.15:
                        points_on_track += 1

        if points_on_track > 50:
            self.lane_change()
        pass


def main():
    try:
        VectorfieldController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("VectorfieldController node terminated.")


if __name__ == '__main__':
    main()
