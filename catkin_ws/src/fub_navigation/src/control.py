#!/usr/bin/env python2
import numpy as np
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from autominy_msgs.msg import NormalizedSteeringCommand, NormalizedSpeedCommand

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, UInt8, Float32, Float64
import rospkg


class VectorfieldController:
    def __init__(self):
        rospy.init_node('VectorfieldController')
        self.map_size_x=600 #cm
        self.map_size_y=430 #cm
        self.resolution = 10 # cm
        self.lane=2
        self.speed_value=0.3
        self.last_angle = -1.0
        print("speed", self.speed_value)
        rospack = rospkg.RosPack()
        self.file_path=rospack.get_path('fub_navigation')+'/src/'
        if (self.lane==1):
            self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')
        else:
            self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')

        self.pub_speed = rospy.Publisher("/control/command/normalized_wanted_speed", NormalizedSpeedCommand, queue_size=100, latch=True)
        rospy.on_shutdown(self.shutdown)

        self.shutdown_=False
        self.pub = rospy.Publisher("/control/command/normalized_wanted_steering", NormalizedSteeringCommand, queue_size=1)
        self.pub_yaw = rospy.Publisher("/desired_yaw", Float32, queue_size=100, latch=True)
        #self.sub_yaw = rospy.Subscriber("/model_car/yaw", Float32, self.callback, queue_size=1)
        #self.sub_odom = rospy.Subscriber("/seat_car/amcl_pose", PoseWithCovarianceStamped, self.callback, queue_size=1)
        self.sub_points = rospy.Subscriber("/clicked_point", PointStamped, self.lane_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber("/localization/corrected_odom", Odometry, self.callback, queue_size=1)


    def lane_callback(self, data):
    	if (self.lane==1):
    		self.lane=2
    		self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')
    	else:
    		self.lane=1
    		self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')

    def callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        x_index_floor = int(math.floor(x * self.resolution))
        y_index_floor = int(math.floor(y * self.resolution))

	x_index_ceil = x_index_floor + 1
        y_index_ceil = y_index_floor + 1

        ceil_ratio_x = x * self.resolution - x_index_floor
        ceil_ratio_y = y * self.resolution - y_index_floor        

        if (x_index_floor < 0):
            x_index_floor = 0
        if (x_index_floor > ((self.map_size_x / self.resolution) - 1)):
            x_index_floor = (self.map_size_x / self.resolution) - 1

        if (y_index_floor < 0):
            y_index_floor = 0
        if (y_index_floor > ((self.map_size_y / self.resolution) - 1)):
            y_index_floor = (self.map_size_y / self.resolution) -1

        if (x_index_ceil < 0):
            x_index_ceil = 0
        if (x_index_ceil > ((self.map_size_x / self.resolution) - 1)):
            x_index_ceil = (self.map_size_x / self.resolution) - 1

        if (y_index_ceil < 0):
            y_index_ceil = 0
        if (y_index_ceil > ((self.map_size_y / self.resolution) - 1)):
            y_index_ceil = (self.map_size_y / self.resolution) - 1

        x3_floor, y3_floor = self.matrix[x_index_floor, y_index_floor, :]
        y3_ceil, y3_ceil = self.matrix[x_index_ceil, y_index_ceil, :]
        x3 = x3_floor * (1.0 - ceil_ratio_x) + x3_floor * ceil_ratio_x
        y3 = y3_floor * (1.0 - ceil_ratio_y) + y3_ceil * ceil_ratio_y
        f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3
        f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3

        Kp = 1.0
	Kd = 0.2
        angle = np.arctan2(f_y, f_x)
        if self.last_angle < 0:
            last_angle = angle

        steering=Kp * angle + Kd * (angle - self.last_angle)
        self.last_angle = angle
        yaw = np.arctan(f_y/(f_x))
        self.pub_yaw.publish(Float32(yaw))

        if (f_x>0):
            speed = -self.speed_value
        else:
            speed = self.speed_value
            if (f_y>0):
            	steering = -np.pi/2
            if (f_y<0):
            	steering = np.pi/2

        if (steering>(np.pi)/2):
            steering = (np.pi)/2

        if (steering<-(np.pi)/2):
            steering = -(np.pi)/2
        if (f_x > 0):
            speed = max(self.speed_value, speed * ((np.pi/3)/(abs(steering)+1)))

        # print(steering)

        steerMsg = NormalizedSteeringCommand()
        steerMsg.value = steering
        self.pub.publish(steerMsg)
        if not self.shutdown_:
            msg = NormalizedSpeedCommand()
            msg.value = speed
            self.pub_speed.publish(msg)

    def shutdown(self):
        print("shutdown!")
        self.shutdown_=True
        msg = NormalizedSpeedCommand()
        msg.value = 0
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
