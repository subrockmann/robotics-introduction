# course: Einführung in die mobile Robotik
# laboratory: 1
# date: 06.11.2020
# author: Susanne Brockmann
# team: 9
# description: transformation and rotation matrices for robotics

import numpy as np
from math import sin, cos

def rot(theta):
    """returns a 2D-rotation matrix with rotation angle theta"""
    R = np.array(((cos(theta), -sin(theta)),
                  (sin(theta), cos(theta))))
    return R

def rotx(theta):
    """returns an elementary 3D-rotation matrix with rotation on axis x by angle theta"""
    R= np.array(((1, 0, 0),
                 (0, cos(theta), -sin(theta)), 
                 (0, sin(theta), cos(theta))))
    return R

def roty(theta):
    """returns an elementary 3D-rotation matrix with rotation on axis y by angle theta"""
    R= np.array(((cos(theta), 0, sin(theta)),
                 (0, 1, 0),
                 (-sin(theta), 0, cos(theta))))
    return R

def rotz(theta):
    """returns an elementary 3D-rotation matrix with rotation on axis z by angle theta"""
    R= np.array(((cos(theta), -sin(theta), 0),
                 (sin(theta), cos(theta), 0), 
                 (0, 0, 1)))
    return R

def rot2trans(r):
    row = np.zeros((1, r.shape[1]))
    R = np.vstack((r, row))
    column = np.zeros((R.shape[0],1))
    R = np.hstack((R, column))
    R[-1,-1] = 1
    return R

def trans(t):
    R = np.zeros((t.shape[0]+1 , t.shape[0]))
    for i in range (0, t.shape[0]):
        R[i,i]=1
    t = np.append(t, 1).reshape(-1, 1)
    R = np.hstack((R, t))
    return R