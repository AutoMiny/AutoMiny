#!/usr/bin/env python

# coding: utf-8
 
# imports
import sys
import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
import pickle

# Message types
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
import xml.etree.cElementTree as ET

root = ET.Element("root")
doc = ET.SubElement(root, "doc")

ET.SubElement(doc, "field1", name="blah").text = "some value1"
ET.SubElement(doc, "field2", name="asdfasd").text = "some vlaue2"

tree = ET.ElementTree(root)
tree.write("filename.xml")

print("done")
angles = [0,30,60,90,105,120,150,180]
def evaluate_lidar(data, set_a=90, sanity=False):

    # convert to x,y
    off = data.angle_min
    inc = data.angle_increment
    l = len(data.ranges)

    # X: Liste von Listen, y Liste voller Nullen
    X = np.ones((l,5))

    # assuming data comes in rad
    for i in range(l):
        a = ((off + i*inc) - np.pi/2) % (2*np.pi)
        #if a < 0:
        #   a = 2*np.pi - a
        # a = (i*inc)
        v = np.array([np.cos(a), np.sin(a)])  # vgl. Kreisformel
        p = data.ranges[i] * v
        X[i,0] = p[0]
        X[i,2] = p[1]
        X[i,3] = data.ranges[i]
        X[i,4] = a
    
    # do some cleanup
    # infinity is to far to fit    
    X = X[X[:,0] != np.inf]
    X = X[X[:,0] != -np.inf]
    
    if sanity:
        # pull a copy for plotting
        Y = X.copy()
    
    # we need to find the fricking table
    dist = 2.5
    # guess the direction of the table from steering angle
    base = np.pi/4 + (np.pi/4)*(set_a/90)
    ang = np.pi/8
    # data should be in front of us
    X = X[X[:,4] > base - ang]
    X = X[X[:,4] < base + ang]
    # and not to far away
    X = X[X[:,3] < dist]
    
    # print(X)
    # fit table as line:   
    a, b = np.linalg.lstsq(X[:,0:2], X[:,2], rcond=-1)[0]

    # get d
    if a == 0:
        alpha = np.pi 
        d = b
    elif a > 0:
        alpha = np.pi/2 - np.arctan(a)
        d = np.sin(alpha) * b
    else:
        alpha = np.pi/2 + np.arctan(a)
        d = np.sin(alpha) * b
    
    # alpha is the smallest angle to the table
    alpha = alpha % np.pi/2
    d = abs(d)
    
    if sanity:
        x = Y[:,0]
        y = Y[:,2]
        plt.scatter(x, y)
        plt.scatter([0], [0],color='r')
        plt.plot(x, a*x + b, 'r')
        ax = plt.gca()
        ci = plt.Circle((0, 0), d, color='b', fill=False)
        ax.add_artist(ci)
        ax.set_xlim((-5, 5))
        ax.set_ylim((-5, 5))
        plt.axes().set_aspect('equal', 'datalim')
        plt.show()
    
    return(a, b, d, alpha)
def calc_wheel_angle(results):
    # this implements the formulas on Sketch 3,4
    l = 0.26        # 26cm in m
    # unpack
    d_01 = results[0][2]
    d_02 = results[1][2]
    # via sum of angles: theta_02 = 180 - 90 - alpha
    theta_02 = np.pi/2 - results[1][3]
    
    # calc
    R = (d_02 - d_01) / np.sin(theta_02)
    gamma = np.arcsin(l/R)
    
    return(R, gamma)
final_data = np.zeros((len(angles), 3))
final_data[:,0] = angles
i = 0
for angle in angles:
    print("\n\n Angle Setting:"+str(angle))
    results = [None, None]
    for set in range(1,3):
        #read file
        filename = "scans/steering_"+str(angle)+"_0"+str(set)+".pkl"
        with open(filename, 'rb') as input:
            data = pickle.load(input)
        # giving the steering angle so we know where to 
        # look for the table 
        if set == 1:
            results[set-1] = evaluate_lidar(data, 90, sanity=True)
        else:
            results[set-1] = evaluate_lidar(data, angle, sanity=True)
   
    print(results)
    
    results2 = calc_wheel_angle(results)
    print(results2)
    
    final_data[i,1:3] = results2
    i += 1
print(final_data)
plt.plot(final_data[:,0], final_data[:,2])
plt.show()
def angle2steering(angle):
    # assume - goes to the left
    STRAIGHT = 105
    
    callib_data = np.array([[   0., 0.56029454, 0.48255242],
                           [  30., 0.62942973,  0.42582508],
                           [  60., 0.71093671,  0.37440057],
                           [  90., 0.82929108,  0.31889851],
                           [ 105., 0.80299232,  0.32973139],
                           [ 120., 0.72895599,  0.36470582],
                           [ 150., 0.61202919,  0.438759  ],
                           [ 180., 0.53409126,  0.508432  ]])
    
    X = np.ones((8,2))
    X[:,0] = callib_data[:,2]
    y = callib_data[:,0]
    
    #fit two straight lines
    a_l, b_l = np.linalg.lstsq(X[0:5,:], y[0:5], rcond=-1)[0]
    a_r, b_r = np.linalg.lstsq(X[5:,:], y[5:], rcond=-1)[0]
    
    if angle < 0:
        steering = a_l * angle + b_l
        if steering < 0: return 0
        elif steering > STRAIGHT: return STRAIGHT
        else: return steering 
    else:
        steering = a_r * angle + b_r
        if steering < STRAIGHT : return STRAIGHT
        elif steering > 180: return 180
        else: return steering 
save_xml()
def save_xml():
	boost_serialization = ET.Element("boost_serialization",signature="serialization::archive",version="12")
	myPair = ET.SubElement(boost_serialization,"myPair",class_id="0",tracking_level="0",version="0")
	count = ET.SubElement(myPair, "count",value=7)
	item_version = ET.SubElement(myPair, "item_version",value=0)
	item = ET.SubElement(myPair, "item")

	for i in range(0,7):
		ET.SubElement(item, "command").text = final_data[i,1]
		ET.SubElement(item, "steering").text = final_data[i,3]

	
	tree = ET.ElementTree(root)
	tree.write("filename.xml")