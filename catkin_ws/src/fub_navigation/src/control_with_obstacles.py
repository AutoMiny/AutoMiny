#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, Point, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int16, Float32
from visualization_msgs.msg import Marker, MarkerArray
from visualization_msgs.msg import InteractiveMarkerInit

class ForceController:
    def __init__(self):

        self.map_size=[600,400] #m
        self.map_origin = [0,0] 
        self.resolution = 10 # cm
        self.obstacle_matrix = np.zeros( (self.map_size[0]/self.resolution+10,self.map_size[1]/self.resolution+10,2),dtype='f' )
        self.lane=1
        self.start=0.0
        if (self.lane==1):
                self.matrix = np.load('matrix130cm_lane1.npy')
        else:
        	self.matrix = np.load('matrixDynamic_lane2.npy')
        self.marker_pub3 = rospy.Publisher('obstacles3', MarkerArray,queue_size=1)
        self.init_markers()
        self.pub = rospy.Publisher("/manual_control/steering", Int16, queue_size=1)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
        self.pub_yaw = rospy.Publisher("/desired_yaw", Float32, queue_size=100, latch=True)
	#self.sub_yaw = rospy.Subscriber("/model_car/yaw", Float32, self.callback, queue_size=1)
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.callback, queue_size=1)
        self.sub_points = rospy.Subscriber("/clicked_point", PointStamped, self.lane_callback, queue_size=1)
        self.sub_ = rospy.Subscriber("/basic_controls/update_full", InteractiveMarkerInit, self.interactive_callback, queue_size=1)
        # Define a marker publisher.


    def interactive_callback(self, data):
		if (data.markers!=[]):
			eps=1
			n = 0
			self.markerArray.markers = []
                        self.obstacle_matrix = np.zeros( (self.map_size[0]/self.resolution+10,self.map_size[1]/self.resolution+10,2),dtype='f' )
                        self.obstacle_matrix.fill(-100)
                        for index in range(0,2):
                                self.obstacle=data.markers[index].pose
                                x1,y1=[data.markers[index].pose.position.x*100,data.markers[index].pose.position.y*100]
                                x_index=np.int(round((x1+self.map_origin[0])/self.resolution))
                                y_index=np.int(round((y1+self.map_origin[1])/self.resolution))
                                if ((x_index>-1)and(x_index<(self.map_size[0]/self.resolution-1))and(y_index>-1)and(y_index<(self.map_size[1]/self.resolution-1))):
                                        for i in range(-10,10):
                                                for j in range(-10,10):
                                                            x_index_=x_index+i
                                                            if (x_index_<0):
                                                                x_index_=0
                                                            if (x_index_>((self.map_size[0]/self.resolution)-1)):
                                                                x_index_=(self.map_size[0]/self.resolution-1)
                                                            y_index_=y_index+j
                                                            if (y_index_<0):
                                                                y_index_=0
                                                            if (y_index_>((self.map_size[1]/self.resolution)-1)):
                                                                y_index_=self.map_size[1]/self.resolution-1
                                                            theta=np.arctan2(self.matrix[x_index,y_index,1],self.matrix[x_index,y_index,0])
                                                            if ((((np.cos(theta)*i+np.sin(theta)*j)**2)/100+((np.sin(theta)*i-np.cos(theta)*j)**2)/10)<1.0):
                                                                Lg= np.sqrt(self.matrix[x_index_,y_index_,0]**2+self.matrix[x_index_,y_index_,1]**2)
                                                                Lo= np.sqrt(i**2+j**2)
                                                                if (Lo!=0):
                                                                    if (self.obstacle_matrix[x_index_,y_index_,0]!=-100):
                                                                        self.obstacle_matrix[x_index_,y_index_,0]=self.obstacle_matrix[x_index_,y_index_,0]+self.matrix[x_index_,y_index_,0]+i*(Lg/Lo)*max(1.0,(4.0/(abs(Lo)+0.1)))
                                                                        self.obstacle_matrix[x_index_,y_index_,1]=self.obstacle_matrix[x_index_,y_index_,1]+self.matrix[x_index_,y_index_,1]+j*(Lg/Lo)*max(1.0,(4.0/(abs(Lo)+0.1)))
                                                                    else:
                                                                        self.obstacle_matrix[x_index_,y_index_,0]=self.matrix[x_index_,y_index_,0]+i*(Lg/Lo)*max(1.0,(3.0/(abs(Lo)+0.1)))
                                                                        self.obstacle_matrix[x_index_,y_index_,1]=self.matrix[x_index_,y_index_,1]+j*(Lg/Lo)*max(1.0,(3.0/(abs(Lo)+0.1)))
                                                                    quaternion = quaternion_from_euler(0, 0, np.arctan2(self.obstacle_matrix[x_index+i,y_index+j,1],2.5*self.obstacle_matrix[x_index+i,y_index+j,0]))
                                                                    marker = Marker()
                                                                    marker.ns = 'obstacles3'
                                                                    marker.id = n
                                                                    marker.type = Marker.ARROW
                                                                    marker.action = Marker.ADD
                                                                    marker.lifetime = rospy.Duration(0)
                                                                    marker.scale.x = 0.12
                                                                    marker.scale.y = 0.03
                                                                    marker.scale.z = 0.04
                                                                    marker.color.r = 0.0
                                                                    marker.color.g = 1.0
                                                                    marker.color.b = 0.0
                                                                    marker.color.a = 1.0
                                                                    pose = Pose()
                                                                    pose.position.x = ((x_index+i)*self.resolution-self.map_origin[0])/100.0;
                                                                    pose.position.y = ((y_index+j)*self.resolution-self.map_origin[1])/100.0;
                                                                    pose.position.z = 0;
                                                                    pose.orientation.x = quaternion[0]
                                                                    pose.orientation.y = quaternion[1]
                                                                    pose.orientation.z = quaternion[2]
                                                                    pose.orientation.w = quaternion[3]
                                                                    marker.header.frame_id = 'map'
                                                                    marker.header.stamp = rospy.Time.now()
                                                                    marker.pose=pose
                                                                    self.markerArray.markers.append(marker)
                                                                    n = n+1
                        self.marker_pub3.publish(self.markerArray)
    def lane_callback(self, data):
#    	if (self.lane==1):
#    		self.lane=2
#    		self.matrix = np.load('matrixDynamic_lane2.npy')
#    	else:
#    		self.lane=1
#    		self.matrix = np.load('matrixDynamic_lane1.npy')
        self.lane=1
        self.start=1.0
        self.speed=-250

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
        if (x_index>((self.map_size[0]/self.resolution)-1)):
            x_index=(self.map_size[0]/self.resolution)-1

        if (y_index<0):
            y_index = 0
        if (y_index>((self.map_size[1]/self.resolution)-1)):
            y_index=(self.map_size[1]/self.resolution)-1

        x3, y3 = self.matrix[x_index,y_index,:]
        f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3

        f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3
        Kp=-4.0
        steering=Kp*np.arctan(f_y/(2.5*f_x))
        if (self.obstacle_matrix[x_index,y_index,0]!=-100):
            x3, y3 = self.obstacle_matrix[x_index,y_index,:]
            f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3
            f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3
            Kp=-4.0
            steering=Kp*np.arctan2(f_y,(2.5*f_x))
        if (f_x>0):
            	speed = -250*self.start
        else:
            speed = 150*self.start
            if (f_y>0):
                steering = np.pi/2
            if (f_y<0):
                steering = -np.pi/2
	self.pub_yaw.publish(Float32(np.arctan(f_y/f_x)))

        if (steering>(np.pi)/2):
            steering = (np.pi)/2

        if (steering<-(np.pi)/2):
            steering = -(np.pi)/2


        steering = 90 + steering * (180/np.pi)
        self.pub.publish(Int16(steering))

        self.pub_speed.publish(Int16(speed))

    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'force_vectors'
        marker_id = 0
        marker_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 0.5}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers', Marker,queue_size=1)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.LINE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale * 0.02
        self.markers.scale.y = marker_scale 
        self.markers.scale.z = marker_scale 
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()


        # Define a marker publisher.
        self.marker_pub1 = rospy.Publisher('force', Marker,queue_size=1)

        # Initialize the marker points list.
        self.markers1 = Marker()
        self.markers1.ns = 'force_vectors2'
        self.markers1.id = 1
        self.markers1.type = Marker.LINE_LIST
        self.markers1.action = Marker.ADD
        self.markers1.lifetime = rospy.Duration(marker_lifetime)
        self.markers1.scale.x = marker_scale * 0.005
        self.markers1.scale.y = marker_scale 
        self.markers1.scale.z = marker_scale 
        self.markers1.color.r = 1.0
        self.markers1.color.g = 0.0
        self.markers1.color.b = 0.0
        self.markers1.color.a = 1.0
        
        self.markers1.header.frame_id = 'map'
        self.markers1.header.stamp = rospy.Time.now()
        self.markers1.points = list()

         # Define a marker publisher.
        self.marker_pub2 = rospy.Publisher('force_filtered', Marker,queue_size=1)

        # Initialize the marker points list.
        self.markers2 = Marker()
        self.markers2.ns = 'force_filtered'
        self.markers2.id = 1
        self.markers2.type = Marker.LINE_LIST
        self.markers2.action = Marker.ADD
        self.markers2.lifetime = rospy.Duration(marker_lifetime)
        self.markers2.scale.x = marker_scale * 0.005
        self.markers2.scale.y = marker_scale 
        self.markers2.scale.z = marker_scale 
        self.markers2.color.r = 0.0
        self.markers2.color.g = 1.0
        self.markers2.color.b = 0.0
        self.markers2.color.a = 1.0
        
        self.markers2.header.frame_id = 'map'
        self.markers2.header.stamp = rospy.Time.now()
        self.markers2.points = list()

        # Initialize the marker points list.
        self.markerArray = MarkerArray()

def log(name, var):
    print '%s:\n%s\n' % (name, var)


def main():
    rospy.init_node('ForceController')
    ForceController()  # constructor creates publishers / subscribers
    rospy.spin()

if __name__ == '__main__':
    main()
