import math
import shapely
from shapely.geometry import LinearRing, LineString, Point
import numpy as np
import random
#import sys, pygame
from matplotlib import pyplot as plt

####### Simulator #######
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

left_wheel_velocity = 0   # robot left wheel velocity in radians/s
right_wheel_velocity = 0  # robot right wheel velocity in radians/s
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
        v_x = np.cos(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2) 
        v_y = np.sin(q)*(R*left_wheel_velocity/2 + R*right_wheel_velocity/2)
        omega = (R*right_wheel_velocity - R*left_wheel_velocity)/(2*L)    
    
        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep


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

Q = np.zeros((stateSpace, actionSpace))

lr = 0.9
gamma = 0.99
epsilon = 0.2

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
    global left_wheel_velocity
    global right_wheel_velocity
    if action == 'F':
        left_wheel_velocity = 0.3
        right_wheel_velocity = 0.3
    elif action == 'B':
        left_wheel_velocity = -0.3
        right_wheel_velocity = -0.3
    elif action == 'L':
        left_wheel_velocity = -0.3
        right_wheel_velocity = 0.3
    elif action == 'R':
        left_wheel_velocity = 0.3
        right_wheel_velocity = -0.3

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


# Simulation loop
#################

for cnt in range(10000):

    #simple single-ray sensor pointing straight forward
    ray = LineString([(x, y), (x+np.cos(q)*scalar,(y+np.sin(q)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)
    distanceWall = np.sqrt((s.x-x)**2+(s.y-y)**2) # Distance wall

    #simple single-ray sensor pointing to the left
    ray0 = LineString([(x, y), (x+np.cos(q+0.4)*scalar,(y+np.sin(q+0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s0 = world.intersection(ray0)
    distanceWall0 = np.sqrt((s0.x-x)**2+(s0.y-y)**2) # Distance wall

    #simple single-ray sensor pointing to the right
    ray4 = LineString([(x, y), (x+np.cos(q-0.4)*scalar,(y+np.sin(q-0.4)*scalar)) ])  # a line from robot to a point outside arena in direction of q
    s4 = world.intersection(ray4)
    distanceWall4 = np.sqrt((s4.x-x)**2+(s4.y-y)**2) # Distance wall


    ##### RF simple controller #####

    if cnt % 500 == 0:
        epsilon -= 0.008

    # Set the percent you want to explore
    if random.uniform(0, 1) < epsilon:
        #Explore: select a random action
        
        # select random number based on action list length (is currently 4)
        rand_num = random.randint(0,3)
        # get action from action list, based on random number
        rand_choice = action_list[rand_num]
       
        # retrieves the current state based on the sensor output s0 and s4
        state = get_state(distanceWall0, distanceWall4)
        # execute action based on the random choice
        do_action(rand_choice)
        # retrieve new state based on sensor outputs
        new_state = get_state(distanceWall0, distanceWall4)

        # Retrieves the index position of the given state
        state_coord = state_list.index(state) 
        # Retrieves the index position of the NEW state
        new_state_coord = state_list.index(new_state)
        # Calculates the best possible action given in the NEW state space
        new_q_max = np.where(Q[new_state_coord] == np.max(Q[new_state_coord]))[0][0]
        # Update Q-table
        Q[state_coord][rand_num] = Q[state_coord][rand_num] + lr * (reward(state, rand_choice) + gamma * new_q_max - Q[state_coord][rand_num])

    else:
        #Exploit: select the action with max value (future reward)

        # Retrieves the current state based on the sensor output s0 and s4
        state = get_state(distanceWall0, distanceWall4)
        # Retrieves the index position of the given state
        state_coord = state_list.index(state)
        # Calculates the best possible action given in a certain state space
        q_max = np.where(Q[state_coord] == np.max(Q[state_coord]))[0][0]
        # Execute the most optimal action
        do_action(action_list[q_max])
        # Retrieves the new state based on the sensor output of s0 and s4
        new_state = get_state(distanceWall0, distanceWall4)
        # Retrieves the index position of the given state
        new_state_coord = state_list.index(new_state)
        # Calculates the best possible action given in a NEW state space (after a new action has been executed)
        new_q_max = np.where(Q[new_state_coord] == np.max(Q[new_state_coord]))[0][0]
        # Updates the Q-table 
        Q[state_coord][q_max] = Q[state_coord][q_max] + lr * (reward(state, action_list[q_max]) + gamma * new_q_max - Q[state_coord][q_max])

    print(epsilon)
    #step simulation
    simulationstep()

    arena_wall = world.distance(Point(x,y))

    #check collision with arena walls 
    if arena_wall<L/2:
        print('*** Collission ***')
        break
        
    #save robot location and sensor rays for matplotlib
    if cnt%50==0:
        robot_pos["x_coord"].append(x)
        robot_pos["y_coord"].append(y)
        robot_pos["Vdir_1"].append(np.cos(q)*0.05)
        robot_pos["Vdir_2"].append(np.sin(q)*0.05)
        robot_pos["sX_coord"].append(s.x)
        robot_pos["sY_coord"].append(s.y)
        robot_pos["s0X_coord"].append(s0.x)
        robot_pos["s0Y_coord"].append(s0.y)
        robot_pos["s4X_coord"].append(s4.x)
        robot_pos["s4Y_coord"].append(s4.y)



############ Matplotlib simulator ############

for i in range(len(robot_pos["x_coord"])):
    plt.axis([-W/2, W/2, -H/2, H/2])

    # robot x,y coordinates
    plt.plot(robot_pos["x_coord"][i],robot_pos["y_coord"][i], marker='.', markersize=10, color="red")   
    
    #mid sensor - 2
    plt.plot([robot_pos["x_coord"][i], robot_pos["sX_coord"][i]], [robot_pos["y_coord"][i], robot_pos["sY_coord"][i]])
    
    #left sensor - 0
    plt.plot([robot_pos["x_coord"][i], robot_pos["s0X_coord"][i]], [robot_pos["y_coord"][i], robot_pos["s0Y_coord"][i]])

    #right sensor - 4
    plt.plot([robot_pos["x_coord"][i], robot_pos["s4X_coord"][i]], [robot_pos["y_coord"][i], robot_pos["s4Y_coord"][i]])

    plt.pause(0.01)
    plt.clf()

print(Q)
plt.show()