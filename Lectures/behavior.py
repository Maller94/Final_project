#!/usr/bin/python3
from threading import Thread
import dbus.mainloop.glib
import dbus
from time import sleep
import matplotlib.pyplot as plt
import os
import numpy as np
import random
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")


class Thymio:
    def __init__(self):
        self.aseba = self.setup()
        self.sensorHorizontalValues = []

    def drive(self, left_wheel_speed, right_wheel_speed):
        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed

        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def sensHorizontal(self):
        while True:
            prox_horizontal = self.aseba.GetVariable(
                "thymio-II", "prox.horizontal")
            self.sensorHorizontalValues = prox_horizontal

############## Bus and aseba setup ######################################

    def setup(self):
        print("Setting up")
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SessionBus()
        asebaNetworkObject = bus.get_object("ch.epfl.mobots.Aseba", "/")

        asebaNetwork = dbus.Interface(
            asebaNetworkObject, dbus_interface="ch.epfl.mobots.AsebaNetwork"
        )
        # load the file which is run on the thymio
        asebaNetwork.LoadScripts(
            "thympi.aesl", reply_handler=self.dbusError, error_handler=self.dbusError
        )

        # scanning_thread = Process(target=robot.drive, args=(200,200,))
        return asebaNetwork

    def stopAsebamedulla(self):
        os.system("pkill -n asebamedulla")

    def dbusReply(self):
        # dbus replys can be handled here.
        # Currently ignoring
        pass

    def dbusError(self, e):
        # dbus errors can be handled here.
        # Currently only the error is logged. Maybe interrupt the mainloop here
        print("dbus error: %s" % str(e))

# ------------------ Main loop here -------------------------

####### Q learning ########


# State space
# s0 - nothing, detect
# s4 - nothing, detect
state_list = ['s0_D_s4_ND', 's0_ND_s4_ND', 's4_D_s0_ND', 's4_D_s0_D']
stateSpace = pow(2, 2)

# action space
# - forward
# - backward
# - left
# - right
action_list = ['F', 'B', 'L', 'R']
actionSpace = len(action_list)

Q = np.zeros((stateSpace, actionSpace))

lr = 0.9
gamma = 0.99
epsilon = 1

def get_state(s0, s4):
    sensors_value = 2750
    if s0 > sensors_value and s4 < sensors_value:
        return state_list[0]
    elif s0 > sensors_value and s4 > sensors_value:
        return state_list[3]
    elif s0 < sensors_value and s4 > sensors_value:
        return state_list[2]
    else:
        return state_list[1]


def do_action(action):
    robot_speed = 200
    if action == 'F':
        left_wheel_velocity = robot_speed+300
        right_wheel_velocity = robot_speed+300
        robot.drive(left_wheel_velocity, right_wheel_velocity)
    elif action == 'B':
        left_wheel_velocity = -robot_speed
        right_wheel_velocity = -robot_speed
        robot.drive(left_wheel_velocity, right_wheel_velocity)
    elif action == 'L':
        left_wheel_velocity = -robot_speed
        right_wheel_velocity = robot_speed
        robot.drive(left_wheel_velocity, right_wheel_velocity)
    elif action == 'R':
        left_wheel_velocity = robot_speed
        right_wheel_velocity = -robot_speed
        robot.drive(left_wheel_velocity, right_wheel_velocity)


def reward(stateParam, actionParam):
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


def main():
    ####### Threads #######
    sensorHorizontalthread = Thread(target=robot.sensHorizontal)
    sensorHorizontalthread.daemon = True
    sensorHorizontalthread.start()

    ####### Avoidance behavior #######
    counter = 0
    while True:
        counter += 1
        global epsilon

        if counter == 50:
            epsilon = 0.1
        
        if counter % 25 == 0:
            print(Q)
        
        try:
            sleep(0.2)
            # Set the percent you want to explore
            if random.uniform(0, 1) < epsilon:
                # Explore: select a random action

                # select random number based on action list length (is currently 4)
                rand_num = random.randint(0, 3)
                # get action from action list, based on random number
                rand_choice = action_list[rand_num]

                # retrieves the current state based on the sensor output s0 and s4
                state = get_state(robot.sensorHorizontalValues[0], robot.sensorHorizontalValues[4])
                # execute action based on the random choice
                do_action(rand_choice)
                # retrieve new state based on sensor outputs
                new_state = get_state(robot.sensorHorizontalValues[0], robot.sensorHorizontalValues[4])

                # Retrieves the index position of the given state
                state_coord = state_list.index(state)
                # Retrieves the index position of the NEW state
                new_state_coord = state_list.index(new_state)
                # Calculates the best possible action given in the NEW state space
                new_q_max = np.where(Q[new_state_coord] == np.max(Q[new_state_coord]))[0][0]
                # Update Q-table
                Q[state_coord][rand_num] = Q[state_coord][rand_num] + lr * (reward(state, rand_choice) + gamma * new_q_max - Q[state_coord][rand_num])

            else:
                # Exploit: select the action with max value (future reward)

                # Retrieves the current state based on the sensor output s0 and s4
                state = get_state(robot.sensorHorizontalValues[0], robot.sensorHorizontalValues[4])
                # Retrieves the index position of the given state
                state_coord = state_list.index(state)
                # Calculates the best possible action given in a certain state space
                q_max = np.where(Q[state_coord] == np.max(Q[state_coord]))[0][0]
                # Execute the most optimal action
                do_action(action_list[q_max])
                # Retrieves the new state based on the sensor output of s0 and s4
                new_state = get_state(robot.sensorHorizontalValues[0], robot.sensorHorizontalValues[4])
                # Retrieves the index position of the given state
                new_state_coord = state_list.index(new_state)
                # Calculates the best possible action given in a NEW state space (after a new action has been executed)
                new_q_max = np.where(Q[new_state_coord] == np.max(Q[new_state_coord]))[0][0]
                # Updates the Q-table
                Q[state_coord][q_max] = Q[state_coord][q_max] + lr * (reward(state, action_list[q_max]) + gamma * new_q_max - Q[state_coord][q_max])
        except:
            sleep(1)
            print("Setting up...")

# ------------------- Main loop end ------------------------


if __name__ == '__main__':
    robot = Thymio()
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping robot")
        robot.stop()
        exit_now = True
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
