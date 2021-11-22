import math
from shapely.geometry import LinearRing, LineString, Point
import numpy as np
import random
#import sys, pygame
from matplotlib import pyplot as plt

####### Simulator #######
# A prototype simulation of a differential-drive robot with 3 front sensors

class Robot:
    def __init__(self,x,y,q):
        # Constants
        ###########
        self.R = 0.021  # radius of wheels in meters
        # circumference is then 2 * 2.1 * pi = 13.1946 CM
        # 0.131946 M
        self.L = 0.095  # distance between wheels in meters
        self.x = x   # robot position in meters - x direction - positive to the right 
        self.y = y   # robot position in meters - y direction - positive up
        self.q = q   # robot heading with respect to x-axis in radians
        self.lw_velocity = 0.3   # robot left wheel velocity in radians/s
        self.rw_velocity = 0.3 # robot right wheel velocity in radians/s

##############################
# Globals
robot_timestep = 0.1 # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01 # timestep in kinematics sim (probably don't touch..)
W = 2.00  # width of arena
H = 2.00  # height of arena
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])
# Scalar value 
scalar = 2*(W + H)

rSeeker = Robot(-0.3,-0.5,0.0)
rAvoider = Robot(0.4,0.2,0.0)

# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep():
    rSeeker.x, rSeeker.y, rSeeker.q
    rAvoider.x, rAvoider.y, rAvoider.q

    for step in range(int(robot_timestep/simulation_timestep)):     #step model time/timestep times
        robot_seek_v_x = np.cos(rSeeker.q)*(rSeeker.R*rSeeker.lw_velocity/2 + rSeeker.R*rSeeker.rw_velocity/2) 
        robot_seek_v_y = np.sin(rSeeker.q)*(rSeeker.R*rSeeker.lw_velocity/2 + rSeeker.R*rSeeker.rw_velocity/2)
        robot_seek_omega = (rSeeker.R*rSeeker.rw_velocity - rSeeker.R*rSeeker.lw_velocity)/(2*rSeeker.L)

        robot_avoid_v_x = np.cos(rAvoider.q)*(rAvoider.R*rAvoider.lw_velocity/2 + rAvoider.R*rAvoider.rw_velocity/2) 
        robot_avoid_v_y = np.sin(rAvoider.q)*(rAvoider.R*rAvoider.lw_velocity/2 + rAvoider.R*rAvoider.rw_velocity/2)
        robot_avoid_omega = (rAvoider.R*rAvoider.rw_velocity - rAvoider.R*rAvoider.lw_velocity)/(2*rAvoider.L)     
    
        rSeeker.x += robot_seek_v_x * simulation_timestep
        rSeeker.y += robot_seek_v_y * simulation_timestep
        rSeeker.q += robot_seek_omega * simulation_timestep

        rAvoider.x += robot_avoid_v_x * simulation_timestep
        rAvoider.y += robot_avoid_v_y * simulation_timestep
        rAvoider.q += robot_avoid_omega * simulation_timestep

for cnt in range(10000):
    #####################
    #### Robot seek #####
    #####################

    #simple single-ray sensor pointing straight forward + 5 degrees
    rSeeker_G0_linestring = LineString([(rSeeker.x, rSeeker.y), (rSeeker.x+np.cos(rSeeker.q - 0.05)*scalar,(rSeeker.y+np.sin(rSeeker.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    rSeeker_G0_intersection = world.intersection(rSeeker_G0_linestring)
    rSeeker_G0 = np.sqrt((rSeeker_G0_intersection.x-rSeeker.x)**2+(rSeeker_G0_intersection.y-rSeeker.y)**2) # Distance wall
    
    #simple single-ray sensor pointing straight forward - 5 degrees
    rSeeker_G1_linestring = LineString([(rSeeker.x, rSeeker.y), (rSeeker.x+np.cos(rSeeker.q + 0.05)*scalar,(rSeeker.y+np.sin(rSeeker.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    rSeeker_G1_intersection = world.intersection(rSeeker_G1_linestring)
    rSeeker_G1 = np.sqrt((rSeeker_G1_intersection.x-rSeeker.x)**2+(rSeeker_G1_intersection.y-rSeeker.y)**2) # Distance wall
    
    # distance between robot_seek & robot_avoid
    # robot_seek_avoidRay = LineString([(robot_seek.x, robot_seek.y), (robot_avoid.x,robot_avoid.y)])  # a line from robot to a point outside arena in direction of q
    
    #####################
    #### Robot avoid ####
    #####################
    #simple single-ray sensor pointing straight forward
    # robot_avoid_frontRay = LineString([(robot_avoid.x, robot_avoid.y), (robot_avoid.x+np.cos(robot_avoid.q)*scalar,(robot_avoid.y+np.sin(robot_avoid.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    # robot_avoid_s = world.intersection(robot_avoid_frontRay)
    # robotAvoid_SensorFront = np.sqrt((robot_avoid_s.x-robot_avoid.x)**2+(robot_avoid_s.y-robot_avoid.y)**2) # Distance wall
    ######################################################################################

    ## Controller robot seek ##

    if rSeeker_G0 < 0.15: 
        rSeeker.lw_velocity = -5
        rSeeker.rw_velocity = 5
    elif rSeeker_G1 < 0.15: 
        rSeeker.lw_velocity = 5
        rSeeker.rw_velocity = -5
    else:
        rSeeker.lw_velocity = 0.3
        rSeeker.rw_velocity = 0.3


    ## Controller robot avoid ##

    # if robotAvoid_SensorFront < 0.5:
    #     robot_avoid.lw_velocity = -3
    #     robot_avoid.rw_velocity = 3
    # else:
    #     robot_avoid.lw_velocity = 0.3
    #     robot_avoid.rw_velocity = 0.3 

    ################################
    simulationstep()
    robot_seek_arena_wall = world.distance(Point(rSeeker.x,rSeeker.y))
    #robot_avoid_arena_wall = world.distance(Point(robot_avoid.x,robot_avoid.y))

    #check collision with arena walls 
    if robot_seek_arena_wall < 0.0001: #or robot_avoid_arena_wall<robot_seek.L/2:
        print('*** Collission ***')
        break
        
    ### Matplotlib Simualtor ###
    if cnt%50==0:
        plt.axis([(-W/2) - 0.1, (W/2) + 0.1, (-H/2) - 0.1, (H/2) + 0.1])

        # arena size
        #LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])
        # making arena walls
        plt.plot([W/2,H/2],[-W/2,H/2]) 
        plt.plot([-W/2,H/2],[-W/2,-H/2])
        plt.plot([-W/2,-H/2],[W/2,-H/2])
        plt.plot([W/2,-H/2],[W/2,H/2])

        ## Robot seek ##
        plt.plot(rSeeker.x,rSeeker.y, marker='.', markersize=15, color="red")   
        #rSeeker ground sensor 0 + 5 degrees
        plt.plot([rSeeker.x, rSeeker_G0_intersection.x], [rSeeker.y, rSeeker_G0_intersection.y])
        #rSeeker ground sensor 1 - 5 degrees
        plt.plot([rSeeker.x, rSeeker_G1_intersection.x], [rSeeker.y, rSeeker_G1_intersection.y])
        ##############################
        
        ## Robot avoid ##
        # plt.plot(robot_avoid.x,robot_avoid.y, marker='.', markersize=15, color="green")   
        # #mid sensor - 2
        # plt.plot([robot_avoid.x, robot_avoid_s.x], [robot_avoid.y, robot_avoid_s.y])
        ##############################

        # Distance between robot_seek & robot_avoid
        # plt.plot([robot_seek.x,robot_avoid.x],[robot_seek.y,robot_avoid.y])

        plt.pause(0.01)
        plt.clf()
plt.show()