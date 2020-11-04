import numpy as np
from math import sin, cos
from transformations import *

# setup test parameters

T_AB_lec = np.array(((-1, 0, 0, -2),
                 (0, -1, 0, 0),
                 (0, 0, 1, 0),
                 (0, 0, 0, 1)))

T_BC_lec = np.array(((0, 1, 0, -4),
                 (-1, 0, 0, -1),
                 (0, 0, 1, 0),
                 (0, 0, 0, 1)))

T_AC_lec = np.array(((0, -1 ,0, 2),
                (1, 0, 0, 1),
                (0, 0, 1, 0),
                (0, 0, 0, 1)))

# helper functions
def print_result(result):
    print(np.round_(result,decimals=2, out=None)) 
    print("x" *20)
    print("\n")
    return

# exercise 2.1.a
print("Exercise 2.1.a")
T_AB = np.matmul(trans(np.array((-2,0,0)).transpose()), rot2trans(rotz(np.radians(180))))
if np.allclose(T_AB, T_AB_lec):  # check for approximate equality of the two arrays

    print("T_AB:")
    print_result(T_AB)

T_BC = np.dot(trans(np.array((-4,-1,0)).transpose()), rot2trans(rotz(np.radians(270))))
if np.allclose(T_BC, T_BC_lec): 
    print("T_BC:")
    print_result(T_BC)

T_AC = np.dot(trans(np.array((2,1,0)).transpose()), rot2trans(rotz(np.radians(90))))
if np.allclose(T_AC, T_AC_lec): 
    print("T_AC:")
    print_result(T_AC)