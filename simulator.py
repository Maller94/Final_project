import math
import shapely
from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
from random import random
import sys, pygame

# A prototype simulation of a differential-drive robot with 3 front sensors

# Constants
###########
R = 0.021  # radius of wheels in meters
# circumference is then 2 * 2.1 * pi = 13.1946 CM
# 0.131946 M
L = 0.095  # distance between wheels in meters

W = 2.00  # width of arena
H = 2.00  # height of arena

robot_timestep = 0.1        # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])

# Variables 
###########

x = 0.0   # robot position in meters - x direction - positive to the right 
y = 0.0   # robot position in meters - y direction - positive up
q = 0.0   # robot heading with respect to x-axis in radians 

left_wheel_velocity = 0.6063   # robot left wheel velocity in radians/s
right_wheel_velocity = 0.6063  # robot right wheel velocity in radians/s
# numbers amount to speed 100 in Thymio

# Scalar value 
scalar = 2*(W + H)

# robot coordinates
robot_pos = {
    "x_coord": [],
    "y_coord": [],
    "Vdir_1": [],
    "Vdir_2": [],
    "sX_coord": [],
    "sY_coord": [],
    "s0X_coord": [],
    "s0Y_coord": [],
    "s4X_coord": [],
    "s4Y_coord": [],
}

# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep():
    global x, y, q

    for step in range(int(robot_timestep/simulation_timestep)):     #step model time/timestep times
        v_x = cos(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2) 
        v_y = sin(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2)
        omega = (R*right_wheel_velocity - R*left_wheel_velocity)/(2*L)    
    
        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep


# Simulation loop
#################

for cnt in range(10000):

    #simple single-ray sensor pointing straight forward
    ray = LineString([(x, y), (x+cos(q)*scalar,(y+sin(q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)
    distanceWall = sqrt((s.x-x)**2+(s.y-y)**2) # Distance wall

    #simple single-ray sensor pointing to the left
    ray0 = LineString([(x, y), (x+cos(q+0.4)*scalar,(y+sin(q+0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s0 = world.intersection(ray0)
    distanceWall0 = sqrt((s0.x-x)**2+(s0.y-y)**2) # Distance wall

    #simple single-ray sensor pointing to the right
    ray4 = LineString([(x, y), (x+cos(q-0.4)*scalar,(y+sin(q-0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s4 = world.intersection(ray4)
    distanceWall4 = sqrt((s4.x-x)**2+(s4.y-y)**2) # Distance wall

    #simple controller - if sensors detect, turn
    turn_rate = 1.2

    if distanceWall4 < 0.20:
        left_wheel_velocity = -turn_rate
        right_wheel_velocity = turn_rate
    elif distanceWall0 < 0.20:
        left_wheel_velocity = turn_rate
        right_wheel_velocity = -turn_rate
    else:                
        left_wheel_velocity = 0.3
        right_wheel_velocity = 0.3
            
    #step simulation
    simulationstep()

    arena_wall = world.distance(Point(x,y))

    #check collision with arena walls 
    if arena_wall<L/2:
        print('Stopped due to collission')
        break
        
    #save robot location and sensor rays and lidar scans for matplotlib
    if cnt%50==0:
        robot_pos["x_coord"].append(x)
        robot_pos["y_coord"].append(y)
        robot_pos["Vdir_1"].append(cos(q)*0.05)
        robot_pos["Vdir_2"].append(sin(q)*0.05)
        robot_pos["sX_coord"].append(s.x)
        robot_pos["sY_coord"].append(s.y)
        robot_pos["s0X_coord"].append(s0.x)
        robot_pos["s0Y_coord"].append(s0.y)
        robot_pos["s4X_coord"].append(s4.x)
        robot_pos["s4Y_coord"].append(s4.y)


############ PyGames simulator ############

