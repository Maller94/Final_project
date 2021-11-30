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

    def receiveInformation(self):
        while True: 
            self.rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")            

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
        scalar = 2
        # to cancel out all small blobs in the color detection
        noise = 1000000
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
            right = np.sum(blue[:,0:frameDivider])
            mid = np.sum(blue[:,frameDivider:(frameDivider*2)])
            left = np.sum(blue[:,(frameDivider*2):])

            # find the maxVal
            maxVal = max([left,right,mid])

            # print('')
            # print('left: '+str(left))
            # print('mid:  '+str(mid))
            # print('right:'+str(right))

            if left == maxVal and left > scalar*mid and left > scalar*right and maxVal > noise:
                self.enemyDirection = 'left'
            elif mid == maxVal and mid > scalar*left and mid > scalar*right and maxVal > noise:
                self.enemyDirection = 'mid'
            elif right == maxVal and right > scalar*mid and right > scalar*left and maxVal > noise:
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

# ------------------ Reinforcement Learning here -------------------------#

# ------------------ Main loop here -------------------------

def main():
    ####### Threads #######
#    sensorGroundThread = Thread(target=robot.sensGround)
#    sensorGroundThread.daemon = True
#    sensorGroundThread.start()

    infraredCommSendThread = Thread(target=robot.sendInformation, args=([1])) ## args=(1) = seeker, args=(2) = avoider
    infraredCommSendThread.daemon = True
    infraredCommSendThread.start()

    infraredCommRecieveThread = Thread(target=robot.receiveInformation)
    infraredCommRecieveThread.daemon = True
    infraredCommRecieveThread.start()

    cameraThread = Thread(target=robot.colorDetection)
    cameraThread.daemon = True
    cameraThread.start()

    # Controller #
    while True:
        try:
            robot.LED('red') 
            #print("0: " + str(robot.sensorGroundValues[0]))
            #print("1: " + str(robot.sensorGroundValues[1]))
            #print(robot.rx[0])
            #robot.LED()
            ####### Basic behavior #######
            # print(robot.enemyDirection)


            if robot.enemyDirection == 'left':
                robot.drive(100,-100)
                print('left')
            elif robot.enemyDirection == 'mid':
                robot.drive(0,0)
                print('mid')
            elif robot.enemyDirection == 'right':
                robot.drive(-100,100)
                print('right')
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
        robot.stopCamera()
        robot.LED('off')
        print("Stopping robot")
        robot.stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

