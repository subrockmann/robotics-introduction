from math import *
import numpy as np
#from Robot_Simulator_V3 import obstacleWorld3
#import stripeWorld#
from Robot_Simulator_V3 import World
from Robot_Simulator_V3 import Robot


# build the world
def buildWorld():
    world = World.World(100, 100)
    steps = 20
    for j in range(0, world._height, steps):    
        for i in range(0, world._width, steps):
            world.addLine( i, j, i+steps/2,  j)

    return world


# Roboter in obstacleWorld3 positionieren:
#myWorld = obstacleWorld3.buildWorld()
myWorld = buildWorld()
myRobot = Robot.Robot()

# Motion noise parameter:
myRobot._k_d = 0 #0.05 * 0.05  # velocity noise parameter = 0.05m*0.05m / 1m
myRobot._k_theta = 0 # (5.0 * 5.0/360.0) * (pi/180.0)  # turning rate noise parameter = 5deg*5deg/360deg * (1rad/1deg)
myRobot._k_drift = 0 # (2.0 * 2.0)/1.0 * (pi/180.0)**2  # drift noise parameter = 2deg*2deg / 1m

myWorld.setRobot(myRobot, [12, myWorld._height/2, 0])

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


straightDrive(myRobot, 1,25 )
turnOnSpot(myRobot, np.radians(90), 1.2)
straightDrive(myRobot, 1,25 )
turnOnSpot(myRobot, np.radians(90), 0.5)
straightDrive(myRobot, 1,25 )
turnOnSpot(myRobot, np.radians(90), 0.5)
straightDrive(myRobot, 1,25 )
turnOnSpot(myRobot, np.radians(90), 0.5)
#straightDrive(myRobot, 0.5, 80)



# Simulation schliessen:
myWorld.close()