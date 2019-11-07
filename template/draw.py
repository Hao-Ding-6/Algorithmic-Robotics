#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Sep 22 18:04:44 2019

@author: dinghao
"""


#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos

# Plot a path in R3 with a unit square obstacle centered at the origin
def plotR2(path):
    fig = plt.figure()
    ax = fig.gca()

    # drawObstacles(ax, obstacle_filename)
#    plotEnv2Obstacles(ax)

    # Plotting the path
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    plt.show()

def plotSE2(obstacle_path, path):
    fig = plt.figure()
    ax = fig.gca()

    drawObstacles(ax, obstacle_filename)
#    plotEnv2Obstacles(ax)

    # Plotting the path (reference point)
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    # Plotting the actual box
    boxVert = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5]]

    for p in path:
        print ("p is:", p)
        x = []
        y = []
        for v in boxVert:
            x.append(v[0] * cos(p[2]) - v[1] * sin(p[2]) + p[0])
            y.append(v[0] * sin(p[2]) + v[1] * cos(p[2]) + p[1])
        ax.plot(x, y, 'k')

    plt.axis([-15,15,-15,15])
    plt.show()

def plotWeird(path):
    fig = plt.figure()
    ax = fig.gca()

    # Translate configuration space path into SE(2) path
    path = [[cos(p[1]) * p[0] - 2, sin(p[1]) * p[0] - 2, p[1]] for p in path]

    plotObstacles(ax)

    # Plotting the path
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    # Plotting the actual box
    boxVert = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5]]

    print ("path is:", path)
    for p in path:
        print ("p is:", p)
        ax.plot((-2,p[0]), (-2,p[1]), color='b', alpha=0.2)
        x = []
        y = []
        for v in boxVert:
            x.append(v[0] * cos(p[2]) - v[1] * sin(p[2]) + p[0])
            y.append(v[0] * sin(p[2]) + v[1] * cos(p[2]) + p[1])
        ax.plot(x, y, 'k')

    plt.show()

def drawObstacles(ax, filename):
    print ("draw obstacle")
    lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]
    data = [[float(x) for x in line.split(' ')] for line in lines[0:]]
    print (data)
    
    for obs in data:
        ax.add_patch(patches.Rectangle((obs[0], obs[1]), obs[2], obs[3], fill='false', color='black'))
    
    # plt.plot(-1.3, -1.3, 'go') # source
    # plt.plot(1.2, 1.2, 'ro') # destination

# Read the cspace definition and the path from filename
def readPath(filename):
    lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]

    if len(lines) == 0:
        print ("That file's empty!")
        sys.exit(1)

    cspace = lines[0].strip()
    if (cspace != 'R2' and cspace != 'SE2' and cspace!= 'Weird'):
        print ("Unknown C-space Identifier: " + cspace)
        sys.exit(1)

    data = [[float(x) for x in line.split(' ')] for line in lines[1:]]
    return cspace, data

if __name__ == '__main__':
    print ("sys args is: ", sys.argv)
    if len(sys.argv) == 2:
        filename = sys.argv[1]
    elif len(sys.argv) == 3:
        obstacle_filename = sys.argv[1]
        filename = sys.argv[2]
    else:
        print ('please input file name')
#        filename = 'path.txt'

    cspace, path = readPath(filename)
    print ("cspace is: ", cspace)
    if cspace == 'R2':
        plotR2(path)
    elif cspace == 'SE2':
        plotSE2(obstacle_filename, path)
    elif cspace == 'Weird':
        plotWeird(path)
