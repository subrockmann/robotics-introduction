from math import *
import numpy as np
#from Robot_Simulator_V3 import obstacleWorld3
#import stripeWorld#
from Robot_Simulator_V3 import World
from Robot_Simulator_V3 import Robot


# build the world
def buildStripeWorld():
    world = World.World(100, 100)
    steps = 20
    for j in range(0, world._height, steps):    
        for i in range(0, world._width, steps):
            world.addLine( i, j, i+steps/2,  j)

    return world

def buildEmptyWorld():
    world = World.World(100, 100)

    return world


# Roboter in obstacleWorld3 positionieren:
#myWorld = obstacleWorld3.buildWorld()
#myWorld = buildStripeWorld()
myWorld = buildEmptyWorld()
myRobot = Robot.Robot()

# Motion noise parameter:
myRobot._k_d = 0 #0.05 * 0.05  # velocity noise parameter = 0.05m*0.05m / 1m
myRobot._k_theta = 0 # (5.0 * 5.0/360.0) * (pi/180.0)  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
myRobot._k_drift = 0 # (2.0 * 2.0)/1.0 * (pi/180.0)**2  # drift noise parameter = 2deg*2deg / 1m



# KeyboardController definieren:
myKeyboardController = myWorld.getKeyboardController()

def straightDrive(robot, v, l):
    t_step = myRobot.getTimeStep()
    print(f'Time step: {t_step}')
    n = int((l/v)/myRobot.getTimeStep()) # time steps  ### hier steckt noch ein Bug drin!!!
    print(n)
    motionList = [[v, 0] for i in range(n)] # movements
    for t in range(n):
        # Bewege Roboter
        motion = motionList[t]
        robot.move(motion)
    return robot

def curveDrive(robot, v, r, theta):
    curve_length = theta * r 
    n = int((curve_length/v)/myRobot.getTimeStep())
    print(n)
    omega = v/r
    if omega >= robot._maxOmega:
        print(f"Speed {v} is too high for driving curves with radius {r}")
        #### TODO drive curves with half the speed
        return
    else:
        motionList = [[v, omega] for i in range(n)] # movements

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

def rectangleDrive():
    straightDrive(myRobot, 1,25 )
    turnOnSpot(myRobot, np.radians(90), 1.2)
    straightDrive(myRobot, 1,25 )
    turnOnSpot(myRobot, np.radians(90), 0.5)
    straightDrive(myRobot, 1,25 )
    turnOnSpot(myRobot, np.radians(90), 0.5)
    straightDrive(myRobot, 1,25 )
    turnOnSpot(myRobot, np.radians(90), 0.5)
    return

def angularDifference(theta_1, theta_2):
    return np.mod((theta_1 - theta_2 + np.pi), 2*np.pi) - np.pi

def getRobotDistanceToPoint(p):
    (robot_x, robot_y,robot_theta) = myWorld.getTrueRobotPose()
    distance_to_point = np.sqrt((p[0]-robot_x)**2 + (p[1]-robot_y)**2)
    print (f'Distance to point2: {distance_to_point}')
    return distance_to_point


def gotoGlobal(robot, v, p, tol):
    K_omega = 0.5   # macht der Wert Sinn?
    K_v = 0.1
    v_max = robot._maxSpeed
    omega_max = robot._maxOmega
    (x_star, y_star) = p

    # determine distance to final position
    (x,y,theta) = myWorld.getTrueRobotPose()
    delta = np.sqrt((x_star - x)**2 + (y_star - y)**2)
    myWorld.addBox(x_star,y_star)

    while delta > tol:     

        print(f"x: {x}, y: {y}")
        theta_star = atan2(y_star -y, x_star - x)
        theta_delta  = angularDifference(theta_star, theta)
        print(f"delta theta: {theta_delta}")
        omega = float(np.minimum(omega_max, K_omega * theta_delta, casting='same_kind')) 
        print(f"Omega: {omega}, {type(omega)} v: {v}, {type(v)}")
        v_t = np.minimum(v_max, v)
        v_t = np.minimum(v_t, K_v * (np.sqrt((x_star - x)**2 + (y_star - y)**2)))
        print(v_t)
        # move the robot for 1 timestep
        robot.move([v_t, omega])

        # determine remaining distance to final position
        (x,y,theta) = myWorld.getTrueRobotPose()
        delta = np.sqrt((x_star - x)**2 + (y_star - y)**2)
    return robot

def line_utils(p1, p2):
    """
    creates useful feature representations of a line through points p1 and p2
    :param p1, p2: points in the format np.array((x, y)).transpose()
    :
    :return: xxx
    """

    #    #define the line p1 - p2
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



def followLine(robot, v, p1, p2, tol= 0.5):
    """
    follow a line with speed v from p1 to p2
    :param robot: robot object
    :param v: velocity of robot
    :param p1: nd.array 2d of line point p1
    :param p2: nd.array 2d of line point p2
    :distance: distance between line and origin
    :return: robot
    """
    v_max = robot._maxSpeed
    omega_max = robot._maxOmega
    K_omega = 0.1
    K_v = 1

    (n, n_norm, n_0, alpha, O_distance) = line_utils(p1, p2)
    
    # draw the polyline 
    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]
    polyline = [[p1_x, p1_y], [p2_x, p2_y]]
    myWorld.drawPolyline(polyline, color ='green')

    # required for calculating the orientation of the line
    betha = np.pi/2 - alpha
    
    while (getRobotDistanceToPoint(p2) > tol):
        # calulate distance between robot and line
        (robot_x, robot_y,robot_theta) = myWorld.getTrueRobotPose()
        distance_robot_line = np.dot(np.array((robot_x, robot_y)).transpose(), n_0) - O_distance
        gamma = atan2(1, distance_robot_line) # vector length of norm vector is 1

        # calculate the correction of angular change
        theta_star = np.pi - betha - gamma
        theta_delta  = angularDifference(theta_star, robot_theta)
        omega_t = np.minimum(omega_max, K_omega * theta_delta)
        v_t = np.minimum(v_max, v)

        # move the robot for 1 timestep
        robot.move([v_t, omega_t])
    
    return robot


# create polyline that needs to be followed
poly = [[[50, 40], [80,80]],
    [[80, 80], [20,80]]]


def followPolyline(robot, v, poly):
    v_max = robot._maxSpeed
    omega_max = robot._maxOmega
    myWorld.drawPolylines(poly, color='green')
    for singleline in poly:
        print(singleline)
        p1 = np.array((singleline[0][0],singleline[0][1])).transpose()
        print(f'p1: {p1}')
        p2 = np.array((singleline[1][0],singleline[1][1])).transpose()
        print(f'p2: {p2}')
        followLine(myRobot, 0.5, p1, p2)

    return robot

myWorld.setRobot(myRobot, [12, myWorld._height/2, 2])
followPolyline(myRobot, 0.5, poly)
#followLine(myRobot, 0.5, np.array((50,40)).transpose(), np.array((80,80)).transpose())
#gotoGlobal(myRobot, 0.5, (23, 50), 1)
#straightDrive(myRobot, 0.5, 30)



# Simulation schliessen:
myWorld.close()