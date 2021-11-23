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
        self.lw_velocity = 0.0   # robot left wheel velocity in radians/s
        self.rw_velocity = 0.0 # robot right wheel velocity in radians/s
        self.boxFrame = LinearRing([(self.x-0.02,self.y-0.02),(self.x+0.02,self.y-0.02),(self.x+0.02,self.y+0.02),(self.x-0.02,self.y+0.02)])

    def updateFrame(self,x,y):
        scale = 0.02
        self.boxFrame = LinearRing([(x-scale,y-scale),(x+scale,y-scale),(x+scale,y+scale),(x-scale,y+scale)])

##############################
# Globals
robot_timestep = 0.1 # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01 # timestep in kinematics sim (probably don't touch..)
W = 2.00  # width of arena
H = 2.00  # height of arena
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])
# Scalar value 
scalar = 2*(W + H)

rSeeker = Robot(0,0,0.0)
rAvoider = Robot(0.5,0.0,0.0)

###### Reinforcement Learning (rSeeker) ######
found = 'NFO'
rSeeker_states = ['FO', 'NFO']
stateSpace = pow(2,1)

rSeeker_actions = ['F','B', 'L', 'R']
actionSpace = len(rSeeker_actions)

Q = np.zeros((stateSpace, actionSpace))

lr = 0.9
gamma = 0.99
epsilon = 0.2

def rSeeker_doAction(action):
    if action == 'F':
        rSeeker.lw_velocity = 0.25
        rSeeker.rw_velocity = 0.25
    elif action == 'B':
        rSeeker.lw_velocity = -0.1
        rSeeker.rw_velocity = -0.1
    elif action == 'L':
        rSeeker.lw_velocity = -0.1
        rSeeker.rw_velocity = 0.1
    elif action == 'R':
        rSeeker.lw_velocity = 0.1
        rSeeker.rw_velocity = -0.1

def reward(stateParam,actionParam):
    if stateParam == 'FO' and actionParam == 'R':
        return 5
    elif stateParam == 'FO' and actionParam == 'L':
        return 5
    elif stateParam == 'FO' and actionParam == 'F':
        return 100
    else:
        return -10



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

        rAvoider.updateFrame(rAvoider.x,rAvoider.y)

for cnt in range(10000):
    #####################
    #### Robot seek #####
    #####################

    # simple single-ray sensor pointing straight forward + 5 degrees
    rSeeker_G0_linestring = LineString([(rSeeker.x, rSeeker.y), (rSeeker.x+np.cos(rSeeker.q - 0.05)*scalar,(rSeeker.y+np.sin(rSeeker.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    rSeeker_G0_intersection = world.intersection(rSeeker_G0_linestring)
    rSeeker_G0 = np.sqrt((rSeeker_G0_intersection.x-rSeeker.x)**2+(rSeeker_G0_intersection.y-rSeeker.y)**2) # Distance wall
    
    # simple single-ray sensor pointing straight forward - 5 degrees
    rSeeker_G1_linestring = LineString([(rSeeker.x, rSeeker.y), (rSeeker.x+np.cos(rSeeker.q + 0.05)*scalar,(rSeeker.y+np.sin(rSeeker.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    rSeeker_G1_intersection = world.intersection(rSeeker_G1_linestring)
    rSeeker_G1 = np.sqrt((rSeeker_G1_intersection.x-rSeeker.x)**2+(rSeeker_G1_intersection.y-rSeeker.y)**2) # Distance wall
    
    # camera detecting avoider robot
    rSeeker_camera_linestring = LineString([(rSeeker.x, rSeeker.y), (rSeeker.x+np.cos(rSeeker.q)*scalar,(rSeeker.y+np.sin(rSeeker.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    rSeeker_camera_intersection_world = world.intersection(rSeeker_camera_linestring)
    try:
        rSeeker_camera_intersection = rSeeker_camera_linestring.intersection(rAvoider.boxFrame)
        rSeeker_camera = np.sqrt((rSeeker_camera_intersection[0].x-rSeeker.x)**2+(rSeeker_camera_intersection[0].y-rSeeker.y)**2) # Distance to avoider robot
        #print(rSeeker_camera)
        found = 'FO'
    except:
        #print('rAvoider not visible')
        found = 'NFO'
    
    #####################
    #### Robot avoid ####
    #####################
    # simple single-ray sensor pointing straight forward + 5 degrees
    rAvoider_G0_linestring = LineString([(rAvoider.x, rAvoider.y), (rAvoider.x+np.cos(rAvoider.q - 0.05)*scalar,(rAvoider.y+np.sin(rAvoider.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    rAvoider_G0_intersection = world.intersection(rAvoider_G0_linestring)
    rAvoider_G0 = np.sqrt((rAvoider_G0_intersection.x-rAvoider.x)**2+(rAvoider_G0_intersection.y-rAvoider.y)**2) # Distance wall
    
    # simple single-ray sensor pointing straight forward - 5 degrees
    rAvoider_G1_linestring = LineString([(rAvoider.x, rAvoider.y), (rAvoider.x+np.cos(rAvoider.q + 0.05)*scalar,(rAvoider.y+np.sin(rAvoider.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    rAvoider_G1_intersection = world.intersection(rAvoider_G1_linestring)
    rAvoider_G1 = np.sqrt((rAvoider_G1_intersection.x-rAvoider.x)**2+(rAvoider_G1_intersection.y-rAvoider.y)**2) # Distance wall
    
    ######################################################################################

    ## Controller robot seek ##

    # if rSeeker_G0 < 0.15: 
    # rSeeker.lw_velocity = -0.1
    # rSeeker.rw_velocity = 0.1
    # elif rSeeker_G1 < 0.15: 
    #     rSeeker.lw_velocity = 5
    #     rSeeker.rw_velocity = -5
    # else:
    #     rSeeker.lw_velocity = 0.3
    #     rSeeker.rw_velocity = 0.3

    if cnt % 1000 == 0:
        epsilon -= 0.008

    # Set the percent you want to explore
    if random.uniform(0, 1) < epsilon:
        #Explore: select a random action
        # select random number based on action list length (is currently 4)
        rand_num = random.randint(0,3)
        # get action from action list, based on random number
        rand_choice = rSeeker_actions[rand_num]
       
        # retrieves the current state based on the sensor output s0 and s4
        state = found
        # execute action based on the random choice
        rSeeker_doAction(rand_choice)
        # retrieve new state based on sensor outputs
        new_state = found

        # Retrieves the index position of the given state
        state_coord = rSeeker_states.index(state) 
        # Retrieves the index position of the NEW state
        new_state_coord = rSeeker_states.index(new_state)
        # Calculates the best possible action given in the NEW state space
        new_q_max = np.where(Q[new_state_coord] == np.max(Q[new_state_coord]))[0][0]
        # Update Q-table
        Q[state_coord][rand_num] = Q[state_coord][rand_num] + lr * (reward(state, rand_choice) + gamma * new_q_max - Q[state_coord][rand_num])

    else:
        #Exploit: select the action with max value (future reward)
        # Retrieves the current state based on the sensor output s0 and s4
        state = found
        # Retrieves the index position of the given state
        state_coord = rSeeker_states.index(state)
        # Calculates the best possible action given in a certain state space
        q_max = np.where(Q[state_coord] == np.max(Q[state_coord]))[0][0]
        # Execute the most optimal action
        rSeeker_doAction(rSeeker_actions[q_max])
        # Retrieves the new state based on the sensor output of s0 and s4
        new_state = found
        # Retrieves the index position of the given state
        new_state_coord = rSeeker_states.index(new_state)
        # Calculates the best possible action given in a NEW state space (after a new action has been executed)
        new_q_max = np.where(Q[new_state_coord] == np.max(Q[new_state_coord]))[0][0]
        # Updates the Q-table 
        Q[state_coord][q_max] = Q[state_coord][q_max] + lr * (reward(state, rSeeker_actions[q_max]) + gamma * new_q_max - Q[state_coord][q_max])

    ## Controller robot avoid ##
    if rAvoider_G0 < 0.25: 
        rAvoider.lw_velocity = -4
        rAvoider.rw_velocity = 4
    elif rAvoider_G1 < 0.25: 
        rAvoider.lw_velocity = 4
        rAvoider.rw_velocity = -4
    else:
        rAvoider.lw_velocity = 0.2
        rAvoider.rw_velocity = 0.2

    ################################
    simulationstep()
    rSeeker_arena_wall = world.distance(Point(rSeeker.x,rSeeker.y))
    rAvoider_arena_wall = world.distance(Point(rAvoider.x,rAvoider.y))
    #robot_avoid_arena_wall = world.distance(Point(robot_avoid.x,robot_avoid.y))

    #check collision with arena walls 
    if rSeeker_arena_wall < 0.0001 or rSeeker_arena_wall < 0.0001: #or robot_avoid_arena_wall<robot_seek.L/2:
        print('*** Collission ***')
        break
    if rSeeker_camera < 0.05:
        print('Robot has been tagged')
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
        #rSeeker camera
        plt.plot([rSeeker.x, rSeeker_camera_intersection_world.x], [rSeeker.y, rSeeker_camera_intersection_world.y])

        ## Robot avoid ##
        plt.plot(rAvoider.x,rAvoider.y, marker='.', markersize=15, color="green")   
        #rSeeker ground sensor 0 + 5 degrees
        plt.plot([rAvoider.x, rAvoider_G0_intersection.x], [rAvoider.y, rAvoider_G0_intersection.y])
        #rSeeker ground sensor 1 - 5 degrees
        plt.plot([rAvoider.x, rAvoider_G1_intersection.x], [rAvoider.y, rAvoider_G1_intersection.y])

        # rAvoide boxFrame
        rAvoider_boxFrame_x,rAvoider_boxFrame_y = rAvoider.boxFrame.xy
        plt.plot(rAvoider_boxFrame_x,rAvoider_boxFrame_y)

        # Distance between robot_seek & robot_avoid

        plt.pause(0.01)
        plt.clf()
plt.show()
print(Q)