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

T_BC = np.matmul(trans(np.array((-4,-1,0)).transpose()), rot2trans(rotz(np.radians(270))))
if np.allclose(T_BC, T_BC_lec): 
    print("T_BC:")
    print_result(T_BC)

T_AC = np.matmul(trans(np.array((2,1,0)).transpose()), rot2trans(rotz(np.radians(90))))
if np.allclose(T_AC, T_AC_lec): 
    print("T_AC:")
    print_result(T_AC)

# exercise 2.1.b
print("Exercise 2.1.b")
T_AC_calc = np.matmul(T_AB,T_BC)
if np.allclose(T_AC_calc, T_AC): 
    print("T_AC:")
    print_result(T_AC)

# exercise 2.1.c
print("Exercise 2.1.c")
I = np.matmul(np.linalg.inv(T_AC), T_AC)
if np.allclose(I ,np.identity(4)):
    print("I:")
    print_result(I)

# exercise 2.1.d
print("Exercise 2.1.d")
p_B = np.array((-3,1, 0, 1)).transpose()
p_A = np.matmul(T_AB, p_B)
print("p_A:")
print_result(p_A)