#!/usr/bin/env python2
import numpy as np

import path_parser

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from scipy.spatial import KDTree
map_size_x=600 #cm
map_size_y=400 #cm
resolution = 10 # cm
matrix = np.zeros( (map_size_x/resolution,map_size_y/resolution,2),dtype='f' )
yaw = -np.pi/3
def main(map_file):
    xy = np.array(list(path_parser.read_points(map_file)))
    x, y = xy.T

    fig = plt.figure(figsize=(12, 10), facecolor='w')
    plt.plot(x, y, ':o', markersize=2)
    global matrix
    matrix = np.load('matrixDynamic.npy')

    plt.gca().set_aspect(1, 'datalim')  # keep circles as circles
    plt.tight_layout()

    def show_nearest(target):

        global yaw
        yaw+=np.pi/3 #np.pi/2
        car_length=0.3

        global matrix
        x1, y1 = target
        print(x1,y1)
        x_index=np.int(x1*resolution)
        y_index=np.int(y1*resolution)
        print(x_index,y_index)
        x3, y3 = matrix[x_index,y_index,:]
        f_x=np.cos(yaw)*x3 + np.sin(yaw)*y3
        f_y=-np.sin(yaw)*x3 + np.cos(yaw)*y3
        Kp=4
        steering=Kp*np.arctan(f_y/(4.0*f_x))

        if (f_x>0):
            speed = -150
        else:
            speed = 150
            if (f_y>0):
                steering = -np.pi/2
            if (f_y<0):
                steering = np.pi/2

        if (steering>(np.pi)/4):
            steering = (np.pi)/4

        if (steering<-(np.pi)/4):
            steering = -(np.pi)/4
        print(steering)

        r = car_length * np.abs(np.tan((np.pi)/2-steering))

        if (r>10):
            r = 10
        print(r)
        if (steering<0.0):
            r=-r
        xc = x1 - np.sin(yaw) * r
        yc = y1 + np.cos(yaw) * r

    


        ax = plt.axes()
        ax.arrow(x1, y1, car_length*np.cos(yaw), car_length*np.sin(yaw), width=car_length, head_width=car_length, head_length=0.09, fc='b', ec='b')

        ax = plt.axes()
        ax.arrow(x1, y1, f_x*np.cos(yaw), f_x*np.sin(yaw), head_width=0.01, head_length=0.01, fc='r', ec='r')

        ax = plt.axes()
        ax.arrow(x1, y1, -f_y*np.sin(yaw), f_y*np.cos(yaw), head_width=0.01, head_length=0.01, fc='r', ec='r')

        ax = plt.axes()
        ax.arrow(x1, y1, x3, y3, head_width=0.01, head_length=0.01, fc='g', ec='g')



        plt.scatter(*target, color='r')
        plt.scatter(*(x1 + x3, y1 + y3), color='g')
        circ = plt.Circle((xc, yc), r, color='r', fill=False)
        plt.gcf().gca().add_artist(circ)
        plt.show(block=False)



    def onclick(event):
        show_nearest((event.xdata, event.ydata))

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
 



if __name__ == '__main__':
    main('sample_map_origin_map.txt')
