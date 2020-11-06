# course: Einführung in die mobile Robotik
# laboratory: 1
# date: 06.11.2020
# author: Susanne Brockmann
# team: 9
# description: exercise 2.1 and 2.2 - rotation and transformation matrices

import numpy as np
from math import sin, cos
from transformations import *


# setup test parameters exercise 2.1
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
    print("x" *25)
    print()
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


# exercise 2.2
# setup test parameters exercise 2.2
T_0A_lec = np.array(((1, 0, 1),
                (0, 1, 1),
                (0, 0, 1)))
            
T_0B_lec = np.array(((0.86, -0.5, 3),
             (0.5, 0.86, 2),
             (0, 0, 1)))

T_AB_lec = np.array(((0.86, -0.5, 2),
             (0.5, 0.86, 1),
             (0, 0, 1)))             

p_0_lec = np.array((3.36, 3.36, 1)).transpose()

p_A_lec = np.array((2.36, 2.36, 1)).transpose()

q_A_lec = np.array((2.87, 4.23, 1)).transpose()



# exercise 2.2.a
print("Exercise 2.2.a")
T_0A = np.matmul(trans(np.array((1,1)).transpose()),rot2trans(rot(np.radians(0))))
if np.allclose(T_0A, T_0A_lec): 
    print("T_0A:")
    print_result(T_0A)
T_0B = np.matmul(trans(np.array((3,2)).transpose()),rot2trans(rot(np.radians(30))))
if np.allclose(T_0B, T_0B_lec): 
    print("T_0B:")
    print_result(T_0B)

# exercise 2.2.b
print("Exercise 2.2.b")
p_B = np.array((1,1,1)).transpose()
p_0 = np.dot(T_0B, p_B)
#if np.allclose(p_0, p_0_lec): # error due to rounding of example answers
print("p_0:")
print_result(p_0)



# exercise 2.2.c
print("Exercise 2.2.c")
T_AB = np.matmul(np.linalg.inv(T_0A), T_0B)
#if np.allclose(T_AB, T_AB_lec): 
print("T_AB:")
print_result(T_AB)

# exercise 2.2.d
print("Exercise 2.2.d")
p_A = np.matmul(T_AB, p_B)
#if np.allclose(p_A, p_A_lec): 
print("p_A:")
print_result(p_A)

# exercise 2.2.e
print("Exercise 2.2.e")
q_A = np.matmul(T_AB, p_A)
#if np.allclose(q_A, q_A_lec): 
print("Rotation of point p_A to generate point q_A:")
print_result(q_A)