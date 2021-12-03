#!/usr/bin/python3
from threading import Thread
import dbus.mainloop.glib
import dbus
from time import sleep
import os
import numpy as np
import random
from picamera import PiCamera
import cv2 as cv2
import test

# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")


class Thymio:
    def __init__(self):
        self.aseba = self.setup()
        self.sensorGroundValues = []
        self.rx = [0]
        #### Infrared communication ####
        #this enables the prox.com communication channels
        self.aseba.SendEventName("prox.comm.enable", [1])
        #This enables the prox.comm rx value to zero, gets overwritten when receiving a value
        #self.aseba.SendEventName("prox.comm.tx",[0])
        self.camera = PiCamera()
        self.enemyDirection = ''

    def drive(self, left_wheel_speed, right_wheel_speed):
        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed

        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def sensGround(self):
        while True:
            prox_ground = self.aseba.GetVariable("thymio-II", "prox.ground.delta")
            self.sensorGroundValues = prox_ground

    def sendInformation(self, number):
        while True: 
            self.aseba.SendEventName("prox.comm.tx", [number])

    ### ONLY TO BE USED IN AVOIDER ###
    def receiveInformation(self):
        while True: 
            if self.aseba.GetVariable("thymio-II", "prox.comm.rx") == [1]:
                self.rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")            
            elif self.aseba.GetVariable("thymio-II", "prox.comm.rx") == [2]:
                self.rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")            
            else: 
                self.rx = [0]

    def LED(self, color): 
        if color == "red": 
            self.aseba.SendEventName("leds.bottom.left", (32,0,0))
            self.aseba.SendEventName("leds.bottom.right", (32,0,0))
            self.aseba.SendEventName("leds.top", (32,0,0))
        elif color == "green": 
            self.aseba.SendEventName("leds.bottom.left", (0,32,0))
            self.aseba.SendEventName("leds.bottom.right", (0,32,0))
            self.aseba.SendEventName("leds.top", (0,32,0))
        elif color == "blue":
            self.aseba.SendEventName("leds.bottom.left", (0,0,32))
            self.aseba.SendEventName("leds.bottom.right", (0,0,32))
            self.aseba.SendEventName("leds.top", (0,0,32))
        elif color == "yellow": 
            self.aseba.SendEventName("leds.bottom.left", (32,32,0))
            self.aseba.SendEventName("leds.bottom.right", (32,32,0))
            self.aseba.SendEventName("leds.top", (32,32,0))
        elif color == "purple": 
            self.aseba.SendEventName("leds.bottom.left", (32,0,32))
            self.aseba.SendEventName("leds.bottom.right", (32,0,32))
            self.aseba.SendEventName("leds.top", (32,0,32))
        else:
            self.aseba.SendEventName("leds.bottom.left", (0,0,0))
            self.aseba.SendEventName("leds.bottom.right", (0,0,0))
            self.aseba.SendEventName("leds.top", (0,0,0))

    def initCamera(self):
        print("Camera test")
        self.camera.start_preview()
        sleep(1)

    def colorDetection(self):
        height = 480
        width = 640
        self.camera.resolution = (height, width)
        self.camera.framerate = 24
        image = np.empty((width, height, 3), dtype=np.uint8)
        #Scalar set to make a greater distinction between left, mid and right
        scalar = 1
        # to cancel out all small blobs in the color detection
        noise = 300000
        noiseCalibration = 200000
        # boundaries blue
        lower_range = np.array([90, 80, 20])
        upper_range = np.array([140, 255, 255])

        while True:
            self.camera.capture(image, 'bgr')
            hsvFrame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
            full_mask = cv2.inRange(hsvFrame, lower_range, upper_range)
            blue = cv2.bitwise_and(image, image, mask=full_mask)

            # To divide the numpy array in three sections
            frameDivider = int(width/3)
            left = np.sum(blue[:,0:frameDivider])
            mid = np.sum(blue[:,frameDivider:(frameDivider*2)])
            right = np.sum(blue[:,(frameDivider*2):])

            # find the maxVal
            maxVal = max([left,right,mid])

            print('')
            if left == maxVal:
                print(f'left: {left} ######')
                print(f'mid:  {mid}')
                print(f'right:{right}')
            elif right == maxVal:
                print(f'left: {left}')
                print(f'mid:  {mid}')
                print(f'right:{right} ######')
            elif mid == maxVal:
                print(f'left: {left}')
                print(f'mid:  {mid} ######')
                print(f'right:{right}')

            if left == maxVal and maxVal > noise-noiseCalibration:
                self.enemyDirection = 'left'
            elif mid == maxVal and maxVal > noise:
                self.enemyDirection = 'mid'
            elif right == maxVal and maxVal > noise+noiseCalibration:
                self.enemyDirection = 'right'
            else:
                self.enemyDirection = 'none'

    def stopCamera(self):
        self.camera.stop_preview()
        

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


## Write q table ##
def writeQ(q): 
    with open("seekerQTable.txt", "w") as write: 
        write.write(q)

# ------------------ Reinforcement Learning here -------------------------#

rSeeker_states = ['left', 'right','mid','none']
stateSpace = len(rSeeker_states)

rSeeker_actions = ['F','B', 'L', 'R']
actionSpace = len(rSeeker_actions)

Q = np.zeros((stateSpace, actionSpace))

lr = 0.9
gamma = 0.99
epsilon = 0.01

def rSeeker_doAction(action):
    if action == 'F':
        robot.drive(450,450)
    elif action == 'B':
        robot.drive(-200,-200)
    elif action == 'L':
        robot.drive(0,300)
    elif action == 'R':
        robot.drive(300,0)

def reward(stateParam,actionParam):
    if stateParam == 'mid' and actionParam == 'F':
        return 100
    elif stateParam == 'left' and actionParam == 'L':
        return 5
    elif stateParam == 'right' and actionParam == 'R':
        return 5
    else:
        return -10



# ------------------ Main loop here -------------------------

def main():
    ####### Threads #######
#    sensorGroundThread = Thread(target=robot.sensGround)
#    sensorGroundThread.daemon = True
#    sensorGroundThread.start()

    infraredCommSendThread = Thread(target=robot.sendInformation, args=([1])) ## args=(1) = seeker, args=(2) = avoider
    infraredCommSendThread.daemon = True
    infraredCommSendThread.start()

    cameraThread = Thread(target=robot.colorDetection)
    cameraThread.daemon = True
    cameraThread.start()

    # Controller #
    while True:
        try:
            robot.LED('red') 
            
            # TASK 1
            # Decrease epsilon slowly.....
            # TASK 2
            # implement basic behavior when detecting tape

            # Set the percent you want to explore
            if random.uniform(0, 1) < epsilon:
                #Explore: select a random action
                # select random number based on action list length (is currently 4)
                rand_num = random.randint(0,3)
                # get action from action list, based on random number
                rand_choice = rSeeker_actions[rand_num]
                # retrieves the current state based on the sensor output s0 and s4
                state = robot.enemyDirection
                # execute action based on the random choice
                rSeeker_doAction(rand_choice)
                # retrieve new state based on sensor outputs
                new_state = robot.enemyDirection

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
                state = robot.enemyDirection
                # Retrieves the index position of the given state
                state_coord = rSeeker_states.index(state)
                # Calculates the best possible action given in a certain state space
                q_max = np.where(Q[state_coord] == np.max(Q[state_coord]))[0][0]
                # Execute the most optimal action
                rSeeker_doAction(rSeeker_actions[q_max])
                # Retrieves the new state based on the sensor output of s0 and s4
                new_state = robot.enemyDirection
                # Retrieves the index position of the given state
                new_state_coord = rSeeker_states.index(new_state)
                # Calculates the best possible action given in a NEW state space (after a new action has been executed)
                new_q_max = np.where(Q[new_state_coord] == np.max(Q[new_state_coord]))[0][0]
                # Updates the Q-table 
                Q[state_coord][q_max] = Q[state_coord][q_max] + lr * (reward(state, rSeeker_actions[q_max]) + gamma * new_q_max - Q[state_coord][q_max])
        except: 
            print("setting up...")
            sleep(1)
# ------------------- Main loop end ------------------------

if __name__ == '__main__':
    robot = Thymio()
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping robot")
        robot.stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
    finally:
        print(Q)
        writeQ(Q)
        robot.stopCamera()
        robot.LED('off')
        print("Stopping robot")
        robot.stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

