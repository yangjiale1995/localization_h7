#!/usr/bin/env python
# coding=utf-8
import sys
import matplotlib.pyplot as plt
import numpy as np
import math

def readfile(filename):
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
    plt.figure('Draw')
    p1=plt.scatter(x1,y1,c='y',marker='s')
    plt.xlabel('X')
    plt.ylabel('Y')
    #plt.xticks(np.arange(-0.1, 0.1, 0.05))
    #plt.yticks(np.arange(-0.1, 0.1, 0.05))
    plt.axis('equal')
    plt.show()
