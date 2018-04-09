#!/usr/bin/env python

# coding: utf-8
 
# imports
import sys
import roslib
import rospy
import numpy as np
import matplotlib.pyplot as plt
import pickle
from scipy import stats


# Message types
from std_msgs.msg import String
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
import xml.etree.cElementTree as ET

angles = [0,30,60,90,120,150,180]
#diactive/active Plotting
sanity=True

def get_distance(points, slope, intercept):
    """ return the distance for each point to the parametrised line """
    pos = np.array((0, intercept))  # origin
    dir = np.array((1, slope))  # line gradient
    # element-wise cross product of each points origin offset with the gradient
    c = np.cross(dir, pos - points, axisb=-1)
    return np.abs(c) / np.linalg.norm(dir)


def get_inliers(points, slope, intercept):
    inlier_dist = 0.05  # max distance for inliers (in meters) since walls are very flat this can be low

    """ return a numpy boolean array for each point (True if within 'inlier_dist' of the line, else False). """
    return get_distance(points, slope, intercept) <= inlier_dist

def find_best_params(xs,ys):
    """ find the best params to describe a detected line using the RANSAC algorithm """
    best_count = 0
    best_params = (0, 0)
    sample_count = 50  # number RANSAC samples to take
    points = np.column_stack((xs, ys))


    # randomly sample points to define a line and remember the one with the most inliers
    for _ in xrange(sample_count):
        ind = np.random.randint(0, len(xs), 2)
        x_a, x_b = xs[ind]
        if x_a == x_b:
            continue  # avoid division by 0

        y_a, y_b = ys[ind].astype(np.float64)

        slope = (y_b - y_a) / (x_b - x_a)
        intercept = y_a - x_a * slope

        inlier = get_inliers(points, slope, intercept)
        inl_count = np.sum(inlier)
        if inl_count > best_count:
            best_count = inl_count
            best_params = (slope, intercept)

    # the set of points within inlier distance of the best line we found
    inlier_points = points[np.where(get_inliers(points, *best_params))]

    # perform a linear regression on the selected inlier points
    slope, intercept, _, _, _ = stats.linregress(*inlier_points.T)

    return slope, intercept

def evaluate_lidar(data, set_a=90, sanity=False):
    global x

    # convert to x,y
    off = data.angle_min  #start angle of the scan [rad]
    inc = data.angle_increment # angular distance between measurements [rad]

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
    
    if sanity:
        # pull a copy for plotting
        Y = X.copy()
    # do some cleanup
    # infinity is to far to fit    
    X = X[X[:,0] != np.inf]
    X = X[X[:,0] != -np.inf]
    
   
    
    # we need to find the fricking table
    dist = 3.1
    # guess the direction of the table from steering angle
    base = 0.83*np.pi #np.pi - ((np.pi/4) + (np.pi/4)*(set_a/90))
    ang =  np.pi/4
    # data should be in front of us
    X = X[X[:,4] > base - ang]
    X = X[X[:,4] < base + ang]
    # and not to far away
    X = X[X[:,3] < dist]

    # if sanity:
    #     # pull a copy for plotting
    #     Y = X.copy()
    

    # print(X)
    # fit table as line:   
    # a, b = np.linalg.lstsq(X[:,0:2], X[:,2], rcond=-1)[0]
    a, b = find_best_params(X[:,0], X[:,2])

    # get d
    d =abs(b)/pow(pow(a,2)+1.0,0.5)
    print(d)
    alpha = np.arctan(a)
    print(b*np.cos(alpha))




    
    if (set_a<90):
    	if (alpha<0):
    		alpha=np.pi+alpha
    if (set_a>91):
        print(set_a)
        if (alpha>0):
            alpha=-np.pi+alpha
    # alpha is the smallest angle to the table
    # alpha = alpha % np.pi/2
    beta=np.pi/2-alpha
    d = abs(d)
    
    if sanity:
        x = Y[:,0]
        y = Y[:,2]
        plt.scatter(x, y)
        plt.scatter([0], [0],color='r')
        plt.plot(x, a*x + b, 'r')

        # plt.plot(x*np.cos(beta)+((-b+x)/a)*np.sin(beta), x*np.sin(beta)-((-b+x)/a)*np.cos(beta), 'g')
        ax = plt.gca()
        ci = plt.Circle((0, 0), d, color='b', fill=False)
        ax.add_artist(ci)
        ax.set_xlim((-5, 5))
        ax.set_ylim((-5, 5))
        plt.title('alpha= %1.3f'%alpha+ ', a= %1.3f'%a)
        plt.axes().set_aspect('equal', 'datalim')
        plt.show()
    
    return(a, b, d, alpha)
def calc_wheel_angle(results,angle,sanity):
    global x
    # this implements the formulas on Sketch 3,4
    l = 0.26        # 26cm in m
    # unpack
    d_01 = results[0][2]
    d_02 = results[1][2]

    # via sum of angles: theta_02 = 180 - 90 - alpha
    theta_01 = results[0][3]
    theta_02 = results[1][3]
    
    
    # calc
    d=abs((d_02*np.sin(theta_01)-d_01*np.sin(theta_02))/(np.sin(theta_01)-np.sin(theta_02)))

    R = abs((d_02 - d) / np.sin(theta_02))

    if (sanity==True):
        if (angle>91):
            R_=R
        else:
            R_=-R
        plt.plot(x,0*x+d, 'g')
        plt.scatter(R_, 0,color='r')
        plt.scatter(R_*(1-np.cos(theta_01)), [d-d_01],color='b')
        plt.scatter(R_*(1-np.cos(theta_02)), [d-d_02],color='k')
        ax = plt.gca()
        ci = plt.Circle((R_, 0), R, color='b', fill=False)
        ax.add_artist(ci)
        ax.set_xlim((-5, 5))
        ax.set_ylim((-5, 5))
        plt.show()

    gamma = np.arcsin(l/R)

    if (angle<91):
   		gamma=-gamma
    
    return(R, gamma)
final_data = np.zeros((len(angles), 3))
final_data[:,0] = angles
i = 0
for angle in angles:
    print("\n\n Angle Setting:"+str(angle))
    results = [None, None]

    for set in range(1,3):
        #read file
        filename = "steering_"+str(angle)+"_0"+str(set)+".pkl"
        with open(filename, 'rb') as input:
            data = pickle.load(input)
        # giving the steering angle so we know where to 
        # look for the table 
        if set == 1:
            results[set-1] = evaluate_lidar(data, 90, sanity=sanity)
        else:
            results[set-1] = evaluate_lidar(data, angle, sanity=sanity)
    results2 = calc_wheel_angle(results,angle,sanity)        
    print(results2)
    

    final_data[i,1:3] = results2
    i += 1
print(final_data)
if (sanity==True):
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
def save_xml():
	
	boost_serialization = ET.Element("boost_serialization",signature="serialization::archive",version="12")
	myPair = ET.SubElement(boost_serialization,"myPair",class_id="0",tracking_level="0",version="0")
	count = ET.SubElement(myPair, "count")
    	count.text=str(7)
	item_version = ET.SubElement(myPair, "item_version")
	item_version.text=str(0)

	for i in range(0,7):
		item = ET.SubElement(myPair, "item")
		ET.SubElement(item, "command").text = str(int(final_data[i,0]))
		ET.SubElement(item, "steering").text = str(final_data[i,2])

	
	tree = ET.ElementTree(boost_serialization)
	file ="SteerAngleActuator.xml"
	file = open(file, "wb")
	file.write("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>\n<!DOCTYPE boost_serialization>\n")
	tree.write(file) 
save_xml()
