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
    x = []
    y=[]
    for i in frange(1.95, 4.03, 0.01):
        yp=0.47
        xp=i
        x.append(xp)
        y.append(yp)
        print(xp, yp)

    xo=4.03
    yo=2.14
    R=1.67
    for i in frange(-np.pi/2.0,np.pi/2.0,1.0/(R*100.0)):
        xp = xo + R * np.cos(i)
        yp = yo + R * np.sin(i)
        y.append(yp)
        x.append(xp)
        print(xp, yp)

    for i in frange(4.03, 1.95, 0.01):
        yp=3.81
        xp=i
        x.append(xp)
        y.append(yp)
        print(xp, yp)

    xo=1.95
    yo=2.14
    R=1.67
    for i in frange(np.pi/2.0,3*np.pi/2.0,1.0/(R*100.0)):
        xp = xo + R * np.cos(i)
        yp = yo + R * np.sin(i)
        y.append(yp)
        x.append(xp)
        print(xp, yp)


    plt.hold(True)
    name=csv_name
    file = open(name,"w")
    for i in range(len(x)):
        if (loop==1):
            file.write('1.1.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')
        if (loop==3):
            file.write('1.2.' + str(format(i+1))+ '\t' + str("{:.3f}".format(x[i]))+ '\t'+str("{:.3f}".format(y[i]))+'\n')

        plt.hold(True)
        plt.plot(x[i],y[i],':o')
    file.close()




def main():
    save("new_map_loop1.txt",1)
    save("new_map_loop2.txt",3)
    plt.show()


if __name__ == '__main__':
    main()
