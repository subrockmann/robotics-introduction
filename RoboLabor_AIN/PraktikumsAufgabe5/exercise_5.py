# Gruppe 9: Carolin König, Susanne Brockmann

from Robot_Simulator_V3 import twoRoomsWorld
from Robot_Simulator_V3 import Robot
from Robot_Simulator_V3 import geometry
from Robot_Simulator_V3.graphics import *
from Robot_Simulator_V3 import sensorUtilities
import numpy as np
from math import pi, sqrt, atan2
#from robot_utils import followLine, followPolyline, followWall, gotoGlobal, wander, turn_on_spot, motion_drive, distance_check



def followLine(robot, v, p1, p2, with_tol=False, tol=0.0):
    """
    :param tol: tolerant value
    :param with_tol: drives to the endpoint of the line with a tolerant value
    :param robot: Robot
    :param v: speed of the robot
    :param p1: start point of the line
    :param p2: end point of the line
    """
    K_omega = 1.0  # Regelparameter für die Berechnung der Winkelgeschwindigkeit omega
    # 1. Gerade erzeugen
    line = ((p1[0], p1[1]), (p2[0], p2[1]))
    # 2. Abstand n zur Geraden berechnen
    x, y, theta = myWorld.getTrueRobotPose()  # aktuelle Position des Roboters
    print("Theta: ", theta * 180 / pi)
    p = (x, y)
    n_vector = geometry.normalToLine(p, line)
    print("Vektor_n", n_vector)
    n_lenght = sqrt(n_vector[0] ** 2 + n_vector[1] ** 2)
    print("Abstandvektor:", n_lenght)
    # # 3. Berechne delta_theta *
    r = ((p2[0] - p1[0]), (p2[1] - p1[1]))
    r_lenght = sqrt(r[0] ** 2 + r[1] ** 2)
    r_norm = ((r[0] / r_lenght), (r[1] / r_lenght))
    print("R normiert:", r_norm)
    point_on_line = ((x + n_vector[0] + r_norm[0]), (y + n_vector[1] + r_norm[1]))
    print("Punkt der angesteuert werden soll:", point_on_line)
    theta_stern = atan2(point_on_line[1] - y, point_on_line[0] - x)

    direction_distance = geometry.dist((x, y), (point_on_line[0], point_on_line[1]))

    print("Theta new", theta_stern * 180 / pi)
    diff = (theta_stern - theta + pi) % (2 * pi) - pi
    omega = min(myRobot.getMaxOmegaValue(), K_omega * diff)
    print("Omega:", omega * 180 / pi)

    n_times = int((omega / omega) / myRobot.getTimeStep())
    print("N:", n_times)
    motionList = [[0.0, omega] for i in range(n_times)]

    # Orientierung auf Gerade zufahren einstellen
    for t in range(n_times):
        motion = motionList[t]
        print(motion)
        robot.move(motion)
        x_t, y_t, theta_t = myWorld.getTrueRobotPose()
        win.plot(x_t, y_t, color='red')

    # Auf Gerade zufahren
    n_times = int((direction_distance / v) / myRobot.getTimeStep())
    motionToLine = [[v, 0.0] for i in range(n_times)]

    for k in range(n_times):
        motion = motionToLine[k]
        print(motion)
        robot.move(motion)
        x_f, y_f, theta_f = myWorld.getTrueRobotPose()
        win.plot(x_f, y_f, color='red')

    # Orientierung auf Gerade fahren einstellen
    curr_x, curr_y, curr_theta = myWorld.getTrueRobotPose()
    theta_stern_end = atan2(p2[1] - curr_y, p2[0] - curr_x)
    diff = (theta_stern_end - curr_theta + pi) % (2 * pi) - pi
    omega_new = min(myRobot.getMaxOmegaValue(), K_omega * diff)
    print("New Omega:", omega_new * 180 / pi)

    n_times = int((omega_new / omega_new) / myRobot.getTimeStep())
    motionRot = [[0.0, omega_new] for i in range(n_times)]

    for j in range(n_times):
        motion = motionRot[j]
        print(motion)
        robot.move(motion)
        x_r, y_r, theta_r = myWorld.getTrueRobotPose()
        win.plot(x_r, y_r, color='red')

    # Auf Linie entlang fahren
    x_Line, y_Line, theta_line = myWorld.getTrueRobotPose()
    print("Theta_line", theta_line * 180 / pi)

    dist_curr_end = geometry.dist((x_Line, y_Line), (p2[0], p2[1]))
    n_times = int((dist_curr_end / v) / myRobot.getTimeStep())
    motionOnLine = [[v, 0.0] for i in range(n_times)]

    for l in range(n_times):
        if with_tol:
            if dist_curr_end < tol:
                break
            else:
                motion = motionOnLine[l]
                robot.move(motion)
                print(motion)
                x_next_line, y_next_line, theta_next_line = myWorld.getTrueRobotPose()
                win.plot(x_next_line, y_next_line, color='red')
                dist_curr_end = geometry.dist((x_next_line, y_next_line), (p2[0], p2[1]))
        else:
            motion = motionOnLine[l]
            robot.move(motion)
            x_next_line, y_next_line, theta_next_line = myWorld.getTrueRobotPose()
            win.plot(x_next_line, y_next_line, color='red')


def gotoGlobal(robot, v, p, tol):
    """
    Robot moves to target point p
    :param robot: Robot
    :param v: speed of the robot
    :param p: target point, the robot should drive to
    :param tol: tol value to reach the point
    """
    K_w = 1.0  # Regelparameter zur Berechnung der Winkelgeschwindigkeit omega
    x, y, theta = myWorld.getTrueRobotPose()
    print("X", x, "Y", y, "Theta alt", theta * 180 / pi)
    theta_new = atan2(p[1] - y, p[0] - x)
    print("New Theta: ", theta_new * 180 / pi)
    diff = ((theta_new - theta + pi) % (2 * pi)) - pi
    print("Diff:", diff * 180 / pi)
    omega = min(myRobot.getMaxOmegaValue(), K_w * diff)
    print("Omega:", omega * 180 / pi)
    delta_t = abs(int((omega / omega) / myRobot.getTimeStep()))
    print("Delta_t:", delta_t)
    motionList = [[0.0, omega] for i in range(delta_t)]

    for t in range(delta_t):
        motion = motionList[t]
        robot.move(motion)
        x_new, y_new, thet_new = myWorld.getTrueRobotPose()
        print(x_new, y_new, thet_new * 180 / pi)
        win.plot(x_new, y_new, color='red')

    dist = geometry.dist((x, y), (p[0], p[1]))
    t = int((dist / v) / myRobot.getTimeStep())  # distance from robot to point
    motionForward = [[v, 0.0] for i in range(t)]
    print(len(motionForward))

    for k in range(t):
        if dist < tol:
            break
        else:
            motion = motionForward[k]
            robot.move(motion)
            x_f, y_f, theta_f = myWorld.getTrueRobotPose()
            print(x_f, y_f)
            win.plot(x_f, y_f, color='red')
            dist = geometry.dist((x_f, y_f), (p[0], p[1]))


def followPolyline(robot, v, poly):
    """
    Robot follows a Polyline
    :param robot: Robot
    :param v: speed of the robot
    :param poly: array of points, they will be connected to a line
    """
    number_of_lines = len(poly)

    followLine(robot, v, poly[0], poly[1], with_tol=True, tol=0.5)  # erste Linie
    for p in range(2, number_of_lines):
        x, y, theta = myWorld.getTrueRobotPose()
        followLine(robot, v, (x, y), poly[p], with_tol=True, tol=0.5)

def wander(robot, v, mode, d, start_theta):
    """
    Robot should drive in the environment without any collisions
    :param d: distance to wall
    :param robot: Robot
    :param mode: choose between driving along a wall or in any way
    :param v: speed of the robot
    """

    theta_big = 100 * pi / 180
    theta_small = 50 * pi / 180
    theta = start_theta

    while True:
        dist = []
        motion = [v, 0.0]
        robot.move(motion)
        x_t, y_t, theta_pose = myWorld.getTrueRobotPose()
        #win.plot(x_t, y_t, color='red')
        lines_g, distance_values = distance_check()
        for dist_value in distance_values:
            if dist_value is not None and dist_value < 1.0:
                dist.append(dist_value)
        if len(dist) > 0:
            min_value = np.amin(dist)
            if min_value < d and mode == 'wall':
                break
            elif min_value < d and len(dist) >= 12 and mode == 'any':
                theta_new = (theta + theta_big)
            elif min_value < d and 5 < len(dist) < 12 and mode == 'any':
                theta_new = (theta + theta_small)
            if min_value < d and (len(dist) > 12 or 5 < len(dist) <= 12) and mode == 'any':
                diff = ((theta_new - theta + pi) % (2 * pi)) - pi
                omega = min(myRobot.getMaxOmegaValue(), diff)
                print("Omega:", omega)
                delta_t = int((omega / omega) / myRobot.getTimeStep())
                motionTurn = [[0.0, omega] for i in range(delta_t)]
                for t in range(delta_t):
                    motion = motionTurn[t]
                    robot.move(motion)
                    x_t, y_t, theta_pose = myWorld.getTrueRobotPose()
                    #win.plot(x_t, y_t, color='red')
                theta = theta_new
            else:
                continue
        else:
            motion = [v, 0.0]
            robot.move(motion)
            x_t, y_t, theta_pose = myWorld.getTrueRobotPose()
            #win.plot(x_t, y_t, color='red')


def turn_on_spot(d, lines, left_rotate=True, first=False):
    """
    Robot makes a turn on the spot
    :param d: distance of the wall
    :param line: wall as line
    :param left_rotate: determines if left rotate or right rotate
    """
    lenght_new_line = sqrt((lines[1][1][0] - lines[1][0][0]) ** 2 + (lines[1][1][1] - lines[1][0][1]) ** 2)
    lenght_old_line = sqrt((lines[0][1][0] - lines[0][0][0]) ** 2 + (lines[0][1][1] - lines[1][0][1]) ** 2)
    alpha_new = atan2(lenght_new_line, d)
    beta_new = atan2(d, lenght_new_line)
    alpha_old = atan2(lenght_old_line, d)
    beta_old = atan2(d, lenght_old_line)
    if first:
        theta_old = 0
    else:
        theta_old = alpha_old + beta_old
    if left_rotate:
        new_theta = alpha_new + beta_new + theta_old
    else:
        new_theta = theta_old - (alpha_new + beta_new)
    print("New Theta: ", new_theta * 180 / pi)
    diff = ((new_theta - theta_old + pi) % (2 * pi)) - pi
    omega = min(myRobot.getMaxOmegaValue(), diff)
    delta_t = int((omega / omega) / myRobot.getTimeStep())
    motionTurn = [[0.0, omega] for i in range(delta_t)]
    for t in range(delta_t):
        motion = motionTurn[t]
        myRobot.move(motion)


def motion_drive(distance, v):
    """
    Robot drives a distance forward
    :param distance: Distance the robot should drive
    :param v: speed of the robot
    """
    times = int((distance / v) / myRobot.getTimeStep())
    motion_to_drive = [[v, 0.0] for i in range(times)]
    for j in range(times):
        motion = motion_to_drive[j]
        myRobot.move(motion)


def distance_check():
    """
    Measure the distances to all walls the robot could see, analyse walls as lines and draw the lines
    :return: walls as lines in array and the distances, measured by the rays
    """
    dists = myRobot.sense()
    directions = myRobot.getSensorDirections()
    lines_l = sensorUtilities.extractSegmentsFromSensorData(dists, directions)
    lines_g = sensorUtilities.transformPolylinesL2G(lines_l, myWorld.getTrueRobotPose())
    myWorld.drawPolylines(lines_g)

    return lines_g, dists


def followWall(v, d):
    """
    Robot follows walls with a distance of d to them
    :param v: speed of the robot
    :param d: distance to the wall
    """
    wall_is_already_drived = False
    rotation_is_done = False

    while True:
        wander(myRobot, v, 'wall', d, 0.0)  # wander, bis Wand am Anfang gefunden wird
        lines_g, dists = distance_check()
        turn_on_spot(d, lines_g, left_rotate=True, first=True)
        while True:
            lines_g, dists = distance_check()
            if len(lines_g) > 1:
                if dists[13] is not None and dists[13] > d:  # dist[13] ist der frontale Strahl, Wand ist in Sicht
                    lines_g, dists = distance_check()
                    first_line = lines_g[0]
                    lenght = sqrt(
                        (first_line[1][0] - first_line[0][0]) ** 2 + (first_line[1][1] - first_line[0][1]) ** 2)
                    time_steps = int((lenght / v) / myRobot.getTimeStep())
                    motion_parallel_to_line = [[v, 0.0] for i in range(time_steps)]
                    for k in range(time_steps):  # Bewegung parallel zur Wand solange bis man auf neues Hindernis trifft
                        if dists[13] is not None and dists[13] > d:
                            if dists[4] is None or dists[5] is None or (
                                    dists[4] > d + 0.1 and dists[5] > d + 0.1):  # dist[4] und [5] liegen
                                # im rechten Winkel zum Roboter, und haben größeren Abstand zur Wand bekommen
                                if not wall_is_already_drived:  # wenn am Ende der Wand,
                                    # laufe noch um d weiter, um den Abstand einzuhalten
                                    motion_drive(d, v)
                                if not rotation_is_done:  # drehe noch in die richtige Richtung
                                    lines_g, dists = distance_check()
                                    turn_on_spot(d, lines_g, left_rotate=False)
                                dists = myRobot.sense()
                                wall_is_already_drived = False
                                rotation_is_done = False
                                if dists[8] is None or dists[8] > d and (
                                        (dists[5] is None or dists[5] > d + 4.0) and dists[7] > d and dists[9] > dists[
                                    8]):  # wenn keine Wand wirklich vorhanden ist,
                                    # dann bewege dich um 2 * d und drehe dich zum weiterfahren,
                                    # ansonsten Bewegung entlang der Wand
                                    motion_drive(2 * d, v)
                                    lines_g, dists = distance_check()
                                    turn_on_spot(d, lines_g, left_rotate=False)

                                lines_g, dists = distance_check()

                                lenght_wall = sqrt(
                                    (lines_g[1][1][0] - lines_g[1][0][0]) ** 2 + (
                                            lines_g[1][1][1] - lines_g[1][0][1]) ** 2)
                                time_steps = int((lenght_wall / v) / myRobot.getTimeStep())
                                motion_wall = [[v, 0.0] for i in range(time_steps)]
                                count_loops = 0
                                for a in range(time_steps):
                                    if count_loops > 20:  # Schritt, der gebraucht wird, wenn Roboter Linie für länger
                                        # hält, als sie in Wirklichkeit ist
                                        if dists[4] is None:
                                            break
                                    motion = motion_wall[a]
                                    myRobot.move(motion)
                                    dists = myRobot.sense()
                                    count_loops = count_loops + 1
                                break
                            else:  # Strahlen, die im rechten Winkel zum Roboter stehen treffen auf Wand
                                motion = motion_parallel_to_line[k]
                                myRobot.move(motion)
                                dists = myRobot.sense()
                                wall_is_already_drived = False
                                rotation_is_done = False
                        else:
                            # Roboter befindet sich im Abstand d vor der Wand, dann einfach in richtige Richtung drehen
                            # bei Innenliegender Ecke
                            lines_g, dists = distance_check()
                            turn_on_spot(d, lines_g, left_rotate=True)
                            break
                elif dists[13] is None:  # keine frontale Wand in Sicht
                    lines_g, dists = distance_check()
                    first_line = lines_g[0]
                    lenght = sqrt(
                        (first_line[1][0] - first_line[0][0]) ** 2 + (first_line[1][1] - first_line[0][1]) ** 2)
                    time_steps = int((lenght / v) / myRobot.getTimeStep())
                    motion_parallel_to_line = [[v, 0.0] for i in range(time_steps)]
                    for k in range(time_steps):
                        if dists[4] is None or dists[5] is None or (
                                dists[4] > d + 0.1 and dists[5] > d + 0.1):  # Abstand zur Wand wird plötzlich größer,
                            # also noch um d weiterfahren und dann drehen, anschließend wieder weiterfahren
                            motion_drive(d, v)
                            lines_g, dists = distance_check()
                            turn_on_spot(d, lines_g, left_rotate=False)
                            dists = myRobot.sense()
                            if dists[6] > d + 1.0:
                                motion_drive(2 * d, v)
                                wall_is_already_drived = True
                            else:
                                wall_is_already_drived = True
                                rotation_is_done = True
                            break
                        else:
                            motion = motion_parallel_to_line[k]  # bewege dich solange entlang dieser Wand,
                            # bis 90° Strahl einen größeren Abstand als d zur Wand bekommt
                            myRobot.move(motion)
                            dists = myRobot.sense()




if __name__ == "__main__":
    # Put robot in twoRoomsWorld:
    myWorld = twoRoomsWorld.buildWorld()
    myRobot = Robot.Robot()

    myWorld.setRobot(myRobot, [10, 6, 0])
    
    wander(myRobot, 0.1, 'any', 0.5, 0.0)

    # Simulation schliessen:
    #myWorld.close()