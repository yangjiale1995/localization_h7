#!/usr/bin/env python
# coding=utf-8
import sys
import matplotlib.pyplot as plt
import numpy as np
import math

i = -0.5

def readfile(filename):
    fp=open(filename)
    data=fp.readlines()

    x=[]
    y=[]

    for line in data:
        yang=line.split('\t')
        x.append(float(yang[1]) * math.cos((132.937 + i) * math.pi / 180.0) - math.sin((132.937 + i) * math.pi / 180.0) * float(yang[2]) + 25.014 - 0.2)
        y.append(float(yang[1]) * math.sin((132.937 + i) * math.pi / 180.0) + math.cos((132.937 + i) * math.pi / 180.0) * float(yang[2]) + 12.3511 + 0.4)
    
    return x,y

def readgps(filename):
    fp=open(filename)
    data=fp.readlines()

    x=[]
    y=[]

    for line in data:
        yang=line.split('\t')
        x.append(float(yang[1]))
        y.append(float(yang[2]))

    return x,y

if __name__ == '__main__':

    x1,y1=readfile(sys.argv[1])
    x2,y2=readgps(sys.argv[2])

    plt.figure('Draw')
    p1=plt.scatter(x1,y1,c='r',marker='s')
    p2=plt.scatter(x2,y2,c='b',marker='s')
    plt.xlabel('X')
    plt.ylabel('Y')
    #plt.xticks(np.arange(-0.1, 0.1, 0.05))
    #plt.yticks(np.arange(-0.1, 0.1, 0.05))
    plt.axis('equal')
    plt.show()
