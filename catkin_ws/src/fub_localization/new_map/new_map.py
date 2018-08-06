#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt,pow
import matplotlib.pyplot as plt

fig = plt.figure()
resolution =0.01
Width=3.85
Length=5.67
R= [1.5,1.1675,0.835]
more_inside=0.03
def frange(x, y, jump):
    if (x<y):
        while x < y:
            yield x
            x += jump
    else:
        while x>y:
            yield x
            x -=jump

def save(csv_name,loop):
    global resolution, Width, Length, R

    dis_edge=(Width/2.0-R[0])
    x = []
    y=[]
    for i in frange(R[0],Length-R[0],resolution):
        yp=dis_edge + (R[0]-R[loop])
        xp=i
        x.append(xp)
        y.append(yp)
        print(xp, yp)

    xo=Length-R[0]-more_inside
    yo=R[0]+dis_edge
    for i in frange(-np.pi/2.0,np.pi/2.0,resolution/R[loop]):
        xp = xo + R[loop] * np.cos(i)
        yp = yo + R[loop] * np.sin(i)
        y.append(yp)
        x.append(xp)
        print(xp, yp)

    for i in frange(Length-R[0],R[0],resolution):          
        xp=i
        yp=dis_edge+R[0]+R[loop]
        x.append(i)
        y.append(yp)
        print(xp, yp)

    xo=R[0]+more_inside
    yo=R[0]+dis_edge
    for i in frange(np.pi/2.0,3*np.pi/2.0,resolution/R[loop]):
        xp = xo + R[loop] * np.cos(i)
        yp = yo + R[loop] * np.sin(i)
        y.append(yp)
        x.append(xp)


    plt.hold(True)
    name=csv_name
    file = open(name,"w")
    for i in range(len(x)):
        file.write('1.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')
        plt.hold(True)
        plt.plot(x[i],y[i],':o')
    file.close()

def save_middle(csv_name,loop):
    global resolution, Width, Length, R
    dis_edge=(Width/2.0-R[0])
    x = []
    y=[]

    for i in frange(4.12,R[0]-resolution,resolution):
        yp=dis_edge + (R[0]-R[loop])
        xp=i
        x.append(xp)
        y.append(yp)
    xo=R[0]
    yo=R[0]+dis_edge
    for i in frange(3*np.pi/2.0,np.pi/2.0,resolution/R[loop]):
        xp = xo + R[loop] * np.cos(i)
        yp = yo + R[loop] * np.sin(i)
        y.append(yp)
        x.append(xp)

    for i in frange(R[0],Length-R[0]+resolution,resolution):          
        xp=i
        yp=dis_edge+R[0]+R[loop]
        x.append(i)
        y.append(yp)
    xo=Length-R[0]
    yo=R[0]+dis_edge
    for i in frange(np.pi/2.0,-np.pi/2.0,resolution/R[loop]):
        xp = xo + R[loop] * np.cos(i)
        yp = yo + R[loop] * np.sin(i)
        y.append(yp)
        x.append(xp)

    plt.hold(True)
    name=csv_name
    file = open(name,"w")
    counter=0
    empty=False
    for i in range(len(x)):
        if (counter<35):
            file.write('1.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')
            plt.hold(True)
            plt.plot(x[i],y[i],':o')    
        else:
            if (empty==False):
                print(x[i-1],y[i-1])
                empty=True
            if (counter>=80):
                counter=-1
                empty=False
        counter=counter+1

    file.close()


def main():
    #save("new_map_outer_loop.txt",0)
    save_middle("new_map_middle_loop.txt",1)
    #save("new_map_inner_loop.txt",2)
    plt.show()


if __name__ == '__main__':
    main()
