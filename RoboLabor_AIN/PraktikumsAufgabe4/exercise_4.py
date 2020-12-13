# author: Susanne Brockmann

from math import *
import random
import numpy as np
from Robot_Simulator_V3 import labyrinthWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import sensorUtilities


# Roboter in Office-World positionieren:
myWorld = labyrinthWorld.buildWorld()
myRobot = Robot.Robot()


# Motion noise parameter:
myRobot._k_d = 0 #0.05 * 0.05  # velocity noise parameter = 0.05m*0.05m / 1m
myRobot._k_theta = 0 # (5.0 * 5.0/360.0) * (pi/180.0)  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
myRobot._k_drift = 0 # (2.0 * 2.0)/1.0 * (pi/180.0)**2  # drift noise parameter = 2deg*2deg / 1m

robot_size = myRobot.getSize()
distance_buffer = 0.2
min_distance = robot_size/2 + distance_buffer
v = 0.5
v_max = myRobot._maxSpeed
omega = 0.1
omega_max = myRobot._maxOmega

K_omega = 0.5
K_v = 1


def turnRandomDirectionOnSpot(robot, theta):
    # randomly set the direction for turning the robo
    numberList = [-1,1]
    direction = random.choice(numberList)
    print(f"Direction: {direction}")
    omega = robot._maxOmega * direction 
    n = int((theta/omega)/myRobot.getTimeStep())
    print (f'Timesteps: {n}')

    # make minimum one move
    if (n == 0):
        n = 1

    motionList = [[0, omega] for i in range(n)] # movements

    for t in range(n):
        # Bewege Roboter
        motion = motionList[t]
        robot.move(motion)
    return robot

def turnOnSpot(robot, theta, omega):
    print()
    if omega >= robot._maxOmega:
        print(f"Rotation speed {omega} is too high for this robot. Max speed {robot._maxOmega}")
        return
    else:
        n = int((theta/omega)/myRobot.getTimeStep())
        print (f'Timesteps: {n}')
        motionList = [[0, omega] for i in range(n)] # movements

        for t in range(n):
            # Bewege Roboter
            motion = motionList[t]
            robot.move(motion)
        return robot

def getObstacleDistance(dists):
    obstacle_dist = min(i for i in list(dists) if i is not None)
    return obstacle_dist

def angularDifference(theta_1, theta_2):
    return np.mod((theta_1 - theta_2 + np.pi), 2*np.pi) - np.pi

def getRobotDistanceToPoint(p):
    (robot_x, robot_y,robot_theta) = myWorld.getTrueRobotPose()
    distance_to_point = np.sqrt((p[0]-robot_x)**2 + (p[1]-robot_y)**2)
    print (f'Distance to point2: {distance_to_point}')
    return distance_to_point

def line_utils(p1, p2):
    """
    creates useful feature representations of a line through points p1 and p2
    :param p1, p2: points in the format np.array((x, y)).transpose()
    :
    :return: xxx
    """
    #define the line p1 - p2
    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]

    # calculate normal vector n for line p1 - p2
    alpha = atan2(p2_y - p1_y, p2_x - p1_x)
    
    n = np.array(((-p2_y + p1_y),(p2_x - p1_x))).transpose()
    n_norm = np.linalg.norm(n)

    if (np.dot(p1, n) >= 0):
        n_0 = n/n_norm
    else:
        n_0 = -n/n_norm
    O_distance = np.dot(p1, n_0)
    #print (f'Distance to origin: {O_distance}')
    #line_direction_vector = np.array(((p2_x - p1_x), (p2_y - p1_y))).transpose()
    #line_direction_vector_norm = np.linalg.norm(line_direction_vector)
    #line_direction_vector_0 = line_direction_vector/line_direction_vector_norm
    return n, n_norm, n_0, alpha, O_distance

def distance_to_line(n_0, O_distance, pose):    
    # calulate distance between robot and line
    (robot_x, robot_y,robot_theta) = pose
    distance_robot_line = np.dot(np.array((robot_x, robot_y)).transpose(), n_0) - O_distance
    return abs(distance_robot_line)

def get_correction_angle(distance_robot_line, betha, pose):
    (robot_x, robot_y,robot_theta) = pose
    # calculate the correction of angular change
    gamma = atan2(1, distance_robot_line) # vector length of norm vector is 1
    betha_gamma = betha + gamma
    #theta_star = angularDifference(np.pi, betha_gamma)
    theta_star = np.pi - betha - gamma
    theta_delta  = angularDifference(theta_star, robot_theta)
    return theta_delta

def moveAlongLine(robot, n_0, O_distance, betha, omega_max, K_omega, v_max, v):
    pose = myWorld.getTrueRobotPose()
    distance_robot_line = distance_to_line(n_0, O_distance, pose)
    theta_delta = get_correction_angle(distance_robot_line, betha, pose)
    #omega_t = np.minimum(omega_max, K_omega * theta_delta)
    omega_t = K_omega * theta_delta
    v_t = np.minimum(v_max, v)
    # move the robot for 1 timestep
    robot.move([v_t, omega_t])
    return robot

def wander(v):
    dists = myRobot.sense()
    while True:
        if (getObstacleDistance(dists) > min_distance):
            myRobot.move([v, 0])
            dists = myRobot.sense()
        else:
            # backup one step
            #myRobot.move([-v, 0])
            # turn randomly
            random_angle = np.random.uniform(low=-np.pi/2, high=np.pi/2)
            turnRandomDirectionOnSpot(myRobot, np.radians(random_angle))
            #turnOnSpot(myRobot, np.radians(random_angle), omega)
            # continue the journey
            myRobot.move([v, 0])
            dists = myRobot.sense()
    return

def followWall(v,d):
    dists = myRobot.sense()
    directions = myRobot.getSensorDirections()
    lines_l = sensorUtilities.extractSegmentsFromSensorData(dists, directions)
    lines_g = sensorUtilities.transformPolylinesL2G(lines_l, myWorld.getTrueRobotPose())
    my_line = lines_g[0]
    myWorld.drawPolyline(my_line)
    print(lines_l[0])
    p1 = lines_l[0][0]
    p2 = lines_l[0][1]
    print(p1)

    (n, n_norm, n_0, alpha, O_distance) = line_utils(p1, p2)

    # required for calculating the orientation of the line
    betha = np.pi/2 - alpha
    pose = myWorld.getTrueRobotPose()
    distance = distance_to_line(n_0, O_distance, pose)
    print(f"Distance to line: {distance}")
    # move the robot along the line
    while (distance_to_line(n_0, O_distance, pose) > d):
        moveAlongLine(myRobot, n_0, O_distance, betha, omega_max, K_omega, v_max, v)   
        pose = myWorld.getTrueRobotPose()

    # while True:
    #     # no line found
    #     if (len(lines_l) == 0):
    #         myRobot.move([v, 0])
    #         dists = myRobot.sense()
    #         directions = myRobot.getSensorDirections()
    #         lines_l = sensorUtilities.extractSegmentsFromSensorData(dists, directions)
    #     else:
    #         line_g = sensorUtilities.transformPolylinesL2G(lines_l[0:1], myWorld.getTrueRobotPose())
    #         myWorld.drawPolylines(line_g)
    return 

myWorld.setRobot(myRobot, [2, 18, 1])

#followWall(0.5, 0.1)





wander(v)

#obstacle_dist =getObstacleDistance(dists)
#print(f"minimal distance to obstacle: {obstacle_dist}")
directions = myRobot.getSensorDirections()

#print(f"dists: {dists} \nl")
#print(f"directions: {directions}")


# Simulation schliessen:
myWorld.close()