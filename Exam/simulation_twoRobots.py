import math
from shapely.geometry import LinearRing, LineString, Point
import numpy as np
import random
#import sys, pygame
from matplotlib import pyplot as plt

####### Simulator #######
# A prototype simulation of a differential-drive robot with 3 front sensors

class Robot:
    def __init__(self,x,y,q,stateSpace,actionSpace):
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
        self.Q = np.zeros((stateSpace, actionSpace))

##############################
# Globals
robot_timestep = 0.1 # 1/robot_timestep equals update frequency of robot
simulation_timestep = 0.01 # timestep in kinematics sim (probably don't touch..)
W = 2.00  # width of arena
H = 2.00  # height of arena
world = LinearRing([(W/2,H/2),(-W/2,H/2),(-W/2,-H/2),(W/2,-H/2)])
# Scalar value 
scalar = 2*(W + H)

####### Q learning ########

# State space
# s0 - nothing, detect (later)
# s4 - nothing, detect (later)
state_list = ['s0_D_s4_ND','s0_ND_s4_ND', 's4_D_s0_ND','s4_D_s0_D']
stateSpace = pow(2,2)

# action space
# - forward
# - backward
# - left (later)
# - right (later)
action_list = ['F','B', 'L', 'R']
actionSpace = len(action_list)

lr = 0.9
gamma = 0.99
epsilon = 0.3

def get_state(s0,s4):
    if s0 < 0.2 and s4 > 0.2:
        return state_list[0]
    elif s0 < 0.2 and s4 < 0.2: 
        return state_list[3]
    elif s0 > 0.2 and s4 < 0.2: 
        return state_list[2]
    else: 
        return state_list[1]
    
def do_action(action):
    robot_seek.lw_velocity
    robot_seek.rw_velocity
    if action == 'F':
        robot_seek.lw_velocity = 0.3
        robot_seek.rw_velocity = 0.3
    elif action == 'B':
        robot_seek.lw_velocity = -0.3
        robot_seek.rw_velocity = -0.3
    elif action == 'L':
        robot_seek.lw_velocity = -0.3
        robot_seek.rw_velocity = 0.3
    elif action == 'R':
        robot_seek.lw_velocity = 0.3
        robot_seek.rw_velocity = -0.3

def reward(stateParam,actionParam):
    if stateParam == 's0_D_s4_ND' and actionParam == 'R':
        return 10
    elif stateParam == 's4_D_s0_ND' and actionParam == 'L':
        return 10
    elif stateParam == 's0_ND_s4_ND' and actionParam == 'F':
        return 100
    elif stateParam == 's0_D_s4_D' and actionParam == 'B':
        return 10
    # elif stateParam == 's0_ND_s4_ND' and actionParam == 'B':
    #     return -100
    else:
        return -10

#### Rewards ####
# forward = 100
# stop = 10
# turn away = 10
# turn into = -10
# backward = -100 

robot_seek = Robot(-0.3,-0.5,0.0,stateSpace,actionSpace)
robot_avoid = Robot(0.4,0.2,0.0,stateSpace,actionSpace)

# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep():
    robot_seek.x, robot_seek.y, robot_seek.q
    robot_avoid.x, robot_avoid.y, robot_avoid.q

    for step in range(int(robot_timestep/simulation_timestep)):     #step model time/timestep times
        robot_seek_v_x = np.cos(robot_seek.q)*(robot_seek.R*robot_seek.lw_velocity/2 + robot_seek.R*robot_seek.rw_velocity/2) 
        robot_seek_v_y = np.sin(robot_seek.q)*(robot_seek.R*robot_seek.lw_velocity/2 + robot_seek.R*robot_seek.rw_velocity/2)
        robot_seek_omega = (robot_seek.R*robot_seek.rw_velocity - robot_seek.R*robot_seek.lw_velocity)/(2*robot_seek.L)

        robot_avoid_v_x = np.cos(robot_avoid.q)*(robot_avoid.R*robot_avoid.lw_velocity/2 + robot_avoid.R*robot_avoid.rw_velocity/2) 
        robot_avoid_v_y = np.sin(robot_avoid.q)*(robot_avoid.R*robot_avoid.lw_velocity/2 + robot_avoid.R*robot_avoid.rw_velocity/2)
        robot_avoid_omega = (robot_avoid.R*robot_avoid.rw_velocity - robot_avoid.R*robot_avoid.lw_velocity)/(2*robot_avoid.L)     
    
        robot_seek.x += robot_seek_v_x * simulation_timestep
        robot_seek.y += robot_seek_v_y * simulation_timestep
        robot_seek.q += robot_seek_omega * simulation_timestep

        robot_avoid.x += robot_avoid_v_x * simulation_timestep
        robot_avoid.y += robot_avoid_v_y * simulation_timestep
        robot_avoid.q += robot_avoid_omega * simulation_timestep

for cnt in range(10000):
    #####################
    #### Robot seek #####
    #####################

    #simple single-ray sensor pointing straight forward
    robot_seek_frontRay = LineString([(robot_seek.x, robot_seek.y), (robot_seek.x+np.cos(robot_seek.q)*scalar,(robot_seek.y+np.sin(robot_seek.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    robot_seek_s = world.intersection(robot_seek_frontRay)
    robotSeek_SensorFront = np.sqrt((robot_seek_s.x-robot_seek.x)**2+(robot_seek_s.y-robot_seek.y)**2) # Distance wall
    #simple single-ray sensor pointing to the left
    robot_seek_leftRay = LineString([(robot_seek.x, robot_seek.y), (robot_seek.x+np.cos(robot_seek.q+0.4)*scalar,(robot_seek.y+np.sin(robot_seek.q+0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    robot_seek_sLeft = world.intersection(robot_seek_leftRay)
    robotSeek_SensorLeft = np.sqrt((robot_seek_sLeft.x-robot_seek.x)**2+(robot_seek_sLeft.y-robot_seek.y)**2) # Distance wall
    #simple single-ray sensor pointing to the right
    robot_seek_rightRay = LineString([(robot_seek.x, robot_seek.y), (robot_seek.x+np.cos(robot_seek.q-0.4)*scalar,(robot_seek.y+np.sin(robot_seek.q-0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    robot_seek_sRight = world.intersection(robot_seek_rightRay)
    robotSeek_SensorRight = np.sqrt((robot_seek_sRight.x-robot_seek.x)**2+(robot_seek_sRight.y-robot_seek.y)**2) # Distance wall

    # distance between robot_seek & robot_avoid
    robot_seek_avoidRay = LineString([(robot_seek.x, robot_seek.y), (robot_avoid.x,robot_avoid.y)])  # a line from robot to a point outside arena in direction of q
    
    #####################
    #### Robot avoid ####
    #####################
    #simple single-ray sensor pointing straight forward
    robot_avoid_frontRay = LineString([(robot_avoid.x, robot_avoid.y), (robot_avoid.x+np.cos(robot_avoid.q)*scalar,(robot_avoid.y+np.sin(robot_avoid.q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    robot_avoid_s = world.intersection(robot_avoid_frontRay)
    robotAvoid_SensorFront = np.sqrt((robot_avoid_s.x-robot_avoid.x)**2+(robot_avoid_s.y-robot_avoid.y)**2) # Distance wall
    #simple single-ray sensor pointing to the left
    robot_avoid_leftRay = LineString([(robot_avoid.x, robot_avoid.y), (robot_avoid.x+np.cos(robot_avoid.q+0.4)*scalar,(robot_avoid.y+np.sin(robot_avoid.q+0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    robot_avoid_sLeft = world.intersection(robot_avoid_leftRay)
    robotAvoid_SensorLeft = np.sqrt((robot_avoid_sLeft.x-robot_avoid.x)**2+(robot_avoid_sLeft.y-robot_avoid.y)**2) # Distance wall
    #simple single-ray sensor pointing to the right
    robot_avoid_rightRay = LineString([(robot_avoid.x, robot_avoid.y), (robot_avoid.x+np.cos(robot_avoid.q-0.4)*scalar,(robot_avoid.y+np.sin(robot_avoid.q-0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    robot_avoid_sRight = world.intersection(robot_avoid_rightRay)
    robotAvoid_SensorRight = np.sqrt((robot_avoid_sRight.x-robot_avoid.x)**2+(robot_avoid_sRight.y-robot_avoid.y)**2) # Distance wall

    ######################################################################################

    ## Controller robot seek ##
    if random.uniform(0, 1) < epsilon:
        #Explore: select a random action
        
        # select random number based on action list length (is currently 4)
        rand_num = random.randint(0,3)
        # get action from action list, based on random number
        rand_choice = action_list[rand_num]
       
        # retrieves the current state based on the sensor output s0 and s4
        state = get_state(robotSeek_SensorLeft, robotSeek_SensorRight)
        # execute action based on the random choice
        do_action(rand_choice)
        # retrieve new state based on sensor outputs
        new_state = get_state(robotSeek_SensorLeft, robotSeek_SensorRight)

        # Retrieves the index position of the given state
        state_coord = state_list.index(state) 
        # Retrieves the index position of the NEW state
        new_state_coord = state_list.index(new_state)
        # Calculates the best possible action given in the NEW state space
        new_q_max = np.where(robot_seek.Q[new_state_coord] == np.max(robot_seek.Q[new_state_coord]))[0][0]
        # Update Q-table
        robot_seek.Q[state_coord][rand_num] = robot_seek.Q[state_coord][rand_num] + lr * (reward(state, rand_choice) + gamma * new_q_max - robot_seek.Q[state_coord][rand_num])

    else:
        #Exploit: select the action with max value (future reward)

        # Retrieves the current state based on the sensor output s0 and s4
        state = get_state(robotSeek_SensorLeft, robotSeek_SensorRight)
        # Retrieves the index position of the given state
        state_coord = state_list.index(state)
        # Calculates the best possible action given in a certain state space
        q_max = np.where(robot_seek.Q[state_coord] == np.max(robot_seek.Q[state_coord]))[0][0]
        # Execute the most optimal action
        do_action(action_list[q_max])
        # Retrieves the new state based on the sensor output of s0 and s4
        new_state = get_state(robotSeek_SensorLeft, robotSeek_SensorRight)
        # Retrieves the index position of the given state
        new_state_coord = state_list.index(new_state)
        # Calculates the best possible action given in a NEW state space (after a new action has been executed)
        new_q_max = np.where(robot_seek.Q[new_state_coord] == np.max(robot_seek.Q[new_state_coord]))[0][0]
        # Updates the Q-table 
        robot_seek.Q[state_coord][q_max] = robot_seek.Q[state_coord][q_max] + lr * (reward(state, action_list[q_max]) + gamma * new_q_max - robot_seek.Q[state_coord][q_max])

    print(robot_seek.Q)
    ## Controller robot avoid ##

    if robotAvoid_SensorFront < 0.3 or robotAvoid_SensorLeft < 0.3 or robotAvoid_SensorRight < 0.3  :
        robot_avoid.lw_velocity = random.uniform(0.3, 1)
        robot_avoid.rw_velocity = -random.uniform(0.3, 1)
    else:
        robot_avoid.lw_velocity = 0.3
        robot_avoid.rw_velocity = 0.3 

    ################################
    simulationstep()
    robot_seek_arena_wall = world.distance(Point(robot_seek.x,robot_seek.y))
    robot_avoid_arena_wall = world.distance(Point(robot_avoid.x,robot_avoid.y))

    #check collision with arena walls 
    if robot_seek_arena_wall<robot_seek.L/2 or robot_avoid_arena_wall<robot_seek.L/2:
        print('*** Collission ***')
        break
        
    ### Matplotlib Simualtor ###
    if cnt%50==0:
        plt.axis([-W/2, W/2, -H/2, H/2])

        ## Robot seek ##
        plt.plot(robot_seek.x,robot_seek.y, marker='.', markersize=15, color="red")   
        #mid sensor - 2
        plt.plot([robot_seek.x, robot_seek_s.x], [robot_seek.y, robot_seek_s.y])
        #left sensor - 0
        plt.plot([robot_seek.x, robot_seek_sLeft.x], [robot_seek.y, robot_seek_sLeft.y])
        #right sensor - 4
        plt.plot([robot_seek.x, robot_seek_sRight.x], [robot_seek.y, robot_seek_sRight.y])
        ##############################
        
        ## Robot avoid ##
        # plt.plot(robot_avoid.x,robot_avoid.y, marker='.', markersize=15, color="green")   
        # #mid sensor - 2
        # plt.plot([robot_avoid.x, robot_avoid_s.x], [robot_avoid.y, robot_avoid_s.y])
        # #left sensor - 0
        # plt.plot([robot_avoid.x, robot_avoid_sLeft.x], [robot_avoid.y, robot_avoid_sLeft.y])
        # #right sensor - 4
        # plt.plot([robot_avoid.x, robot_avoid_sRight.x], [robot_avoid.y, robot_avoid_sRight.y])
        ##############################

        # Distance between robot_seek & robot_avoid
        plt.plot([robot_seek.x,robot_avoid.x],[robot_seek.y,robot_avoid.y])

        plt.pause(0.01)
        plt.clf()
plt.show()