#!/usr/bin/env python2
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, UInt8, Float32
import rospkg


class VectorfieldController:
    def __init__(self):
        rospy.init_node('VectorfieldController')
        self.map_size_x=600 #cm
        self.map_size_y=400 #cm
        self.resolution = 10 # cm
        self.lane=1
        self.speed_value= 300
        rospack = rospkg.RosPack()
        file_path=rospack.get_path('fub_navigation')+'/src/'
        if (self.lane==1):
            self.matrix = np.load(file_path+'matrix50cm_lane1.npy')
        else:
            self.matrix = np.load(file_path+'matrix100cm_lane2.npy')

        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
        rospy.on_shutdown(self.shutdown)

        self.shutdown_=False
        self.pub = rospy.Publisher("/steering", UInt8, queue_size=1)
        self.pub_yaw = rospy.Publisher("/desired_yaw", Float32, queue_size=100, latch=True)
        #self.sub_yaw = rospy.Subscriber("/model_car/yaw", Float32, self.callback, queue_size=1)
        #self.sub_odom = rospy.Subscriber("/seat_car/amcl_pose", PoseWithCovarianceStamped, self.callback, queue_size=1)
        self.sub_points = rospy.Subscriber("/clicked_point", PointStamped, self.lane_callback, queue_size=1)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback, queue_size=1)


    def lane_callback(self, data):
    	if (self.lane==1):
    		self.lane=2
    		self.matrix = np.load(file_path+'matrix100cm_lane2.npy')
    	else:
    		self.lane=1
    		self.matrix = np.load(file_path+'matrix100cm_lane1.npy')

    def callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        x_index=np.int(x*self.resolution)
        y_index=np.int(y*self.resolution)
        
        if (x_index<0):
            x_index = 0
        if (x_index>((self.map_size_x/self.resolution)-1)):
            x_index=(self.map_size_x/self.resolution)-1

        if (y_index<0):
            y_index = 0
        if (y_index>((self.map_size_y/self.resolution)-1)):
            y_index=(self.map_size_y/self.resolution)-1

        x3, y3 = self.matrix[x_index,y_index,:]
        f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3

        f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3
        Kp=4.0
        steering=Kp*np.arctan(f_y/(2.5*f_x))
        yaw = np.arctan(f_y/(f_x))
        self.pub_yaw.publish(Float32(yaw))

        if (f_x>0):
            speed = self.speed_value
        else:
            speed = -self.speed_value
            if (f_y>0):
            	steering = -np.pi/2
            if (f_y<0):
            	steering = np.pi/2

        if (steering>(np.pi)/2):
            steering = (np.pi)/2

        if (steering<-(np.pi)/2):
            steering = -(np.pi)/2
        if (f_x > 0):
            speed = max(300, speed * ((np.pi/3)/(abs(steering)+1)))


        steering = 90 + steering * (180/np.pi)
        # print(steering)

        self.pub.publish(UInt8(steering))
        if not self.shutdown_:
            self.pub_speed.publish(Int16(speed))

    def shutdown(self):
        print("shutdown!")
        self.shutdown_=True
        self.pub_speed.publish(Int16(0))
        rospy.sleep(1)

def main():
    try:
        VectorfieldController() 
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("VectorfieldController node terminated.")


if __name__ == '__main__':
    main()
