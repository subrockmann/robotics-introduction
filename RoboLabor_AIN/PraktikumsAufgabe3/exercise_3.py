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
    K_omega = 0.1   # macht der Wert Sinn?
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

def followLine(robot, v, p1, p2):
    """
    follow a line with speed v from p1 to p2
    :param robot: robot object
    :param v: velocity of robot
    :param p1: nd.array 2d of line point p1
    :param p2: nd.array 2d of line point p2
    :distance: distanc
    :return: robot
    """
    v_max = robot._maxSpeed
    omega_max = robot._maxOmega
    K_omega = 0.1

    #define the line p1 - p2
    p1_x = p1[0]
    p1_y = p1[1]
    p2_x = p2[0]
    p2_y = p2[1]
    #(p1_x, p1_y) = p1
    #(p2_x, p2_y) = p2

    polyline = [[p1_x, p1_y], [p2_x, p2_y]]
    myWorld.drawPolyline(polyline, color ='green')

    # calculate normal vector n for line p1 - p2
    alpha = atan2(p2_y - p1_y, p2_x - p1_x)
    betha = np.pi/2 - alpha
    
    n = np.array(((-p2_y + p1_y),(p2_x - p1_x))).transpose()
    n_norm = np.linalg.norm(n)
    a = np.dot(p1, n)
    if (np.dot(p1, n) >= 0):
        n_0 = n/n_norm
    else:
        n_0 = -n/n_norm
    distance = np.dot(p1, n_0)
    print (f'Distance to origin: {distance}')
    line_direction_vector = np.array(((p2_x - p1_x), (p2_y - p1_y))).transpose()
    line_direction_vector_norm = np.linalg.norm(line_direction_vector)
    line_direction_vector_0 = line_direction_vector/line_direction_vector_norm

    while (getRobotDistanceToPoint(p2)>1):
        # calulate distance between robot and line
        (robot_x, robot_y,robot_theta) = myWorld.getTrueRobotPose()
        distance_robot_line = np.dot(np.array((robot_x, robot_y)).transpose(), n_0) - distance
        #theta_star = atan2(1, distance_robot_line) ### hier steckt noch der Wurm drin - np.radians(90) # vector length of norm vector is 1
        gamma = atan2(1, distance_robot_line)

        # calculate the correction of angular change
        theta_star = np.pi - betha - gamma
        theta_delta  = angularDifference(theta_star, robot_theta)
        omega = np.minimum(omega_max, K_omega * theta_delta)
        v_t = np.minimum(v_max, v)

        # move the robot for 1 timestep
        robot.move([v_t, omega])
    
    return robot

myWorld.setRobot(myRobot, [12, myWorld._height/2, 2])
followLine(myRobot, 0.5, np.array((50,40)).transpose(), np.array((80,80)).transpose())
#gotoGlobal(myRobot, 0.5, (23, 50), 1)
#straightDrive(myRobot, 0.5, 30)



# Simulation schliessen:
myWorld.close()