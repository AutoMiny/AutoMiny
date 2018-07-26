#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import numpy as np
from math import sqrt,pow
import matplotlib.pyplot as plt

fig = plt.figure()
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
    Width=3.85
    Length=5.67
    R1= 1.5
    R2=0.8
    x = []
    y=[]
    dis_edge=(Width/2.0-R1)
    for i in frange(R1,Length-R1,0.1):
        yp=dis_edge+loop*(R1-R2)/4.0
        xp=i
        x.append(xp)
        y.append(yp)
        print(xp, yp)

    xo=Length-R1
    yo=R1+dis_edge
    R=R1-loop*((R1-R2)/4.0)
    for i in frange(-np.pi/2.0,np.pi/2.0,0.1/R):
        xp = xo + R * np.cos(i)
        yp = yo + R * np.sin(i)
        y.append(yp)
        x.append(xp)
        print(xp, yp)

    for i in frange(Length-R1,R1,0.1):          
        xp=i
        yp=dis_edge+R1+(R1-loop*(R1-R2)/4.0)
        x.append(i)
        y.append(yp)
        print(xp, yp)

    xo=R1
    yo=R1+dis_edge
    for i in frange(np.pi/2.0,3*np.pi/2.0,0.1/R):
        xp = xo + R * np.cos(i)
        yp = yo + R * np.sin(i)
        y.append(yp)
        x.append(xp)


    plt.hold(True)
    name=csv_name
    file = open(name,"w")
    for i in range(len(x)):
        file.write('1.1.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')
        plt.hold(True)
        plt.plot(x[i],y[i],':o')
    file.close()




def main():
    save("new_map_loop1.txt",1)
    plt.show()


if __name__ == '__main__':
    main()
