#!/usr/bin/python3
from threading import Thread
import dbus.mainloop.glib
import dbus
from time import sleep
import os
import numpy as np
import random
from math import floor
from adafruit_rplidar import RPLidar

# initialize asebamedulla in background and wait 1s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 1")


class Thymio:
    def __init__(self):
        self.aseba = self.setup()
        self.PORT_NAME = '/dev/ttyUSB0'
        self.sensorGroundValues = []
        self.rx = [0]
        self.closestBeam = "none"
        #### Lidar ####
        self.lidar = RPLidar(None, self.PORT_NAME)
        # This is where we store the lidar readings
        self.scan_data = [0]*360
        self.exit_now = False
        self.robotState = 'drive'
        self.lock = False

        #### Infrared communication ####
        #this enables the prox.com communication channels
        self.aseba.SendEventName("prox.comm.enable", [1])
        #This enables the prox.comm rx value to zero, gets overwritten when receiving a value
        #self.aseba.SendEventName("prox.comm.tx",[0])
        

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
            self.aseba.SendEventName("prox.comm.enable", [1])
            sleep(0.2)
            if self.aseba.GetVariable("thymio-II", "prox.comm.rx") == [1]:
                self.rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")            
            elif self.aseba.GetVariable("thymio-II", "prox.comm.rx") == [2]:
                self.rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")            
            else: 
                self.rx = [0]
            sleep(0.2)
            self.aseba.SendEventName("prox.comm.enable", [0])           

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
    
    def disable_sensor_leds(self):
        self.aseba.SendEventName("leds.temperature", [0])
        self.aseba.SendEventName("leds.circle", [0])
        self.aseba.SendEventName("leds.prox.h", [0, 0, 0, 0, 0])
        self.aseba.SendEventName("leds.prox.v", [0, 0])
        self.aseba.SendEventName("leds.buttons", [0])
        self.aseba.SendEventName("leds.sound", [0])
        self.aseba.SendEventName("leds.rc", [0])

    ############## LIDAR ###############################
    def lidar_scan(self):
        radius = 500
        for scan in self.lidar.iter_scans():
            if(self.exit_now):
               return
            for (_, angle, distance) in scan:
                if distance > radius:
                    self.scan_data[min([359, floor(angle)])] = radius
                else:
                    self.scan_data[min([359, floor(angle)])] = distance

    def lidar_stop(self):
        self.lidar.stop()
        self.lidar.disconnect()
        self.exit_now = True

    def detection(self):
        frontBeam = self.scan_data[135:225] 
        rightBeam = self.scan_data[225:315] 
        backBeam = self.scan_data[315:] + self.scan_data[0:45]
        leftBeam = self.scan_data[45:135] 
        
        if sum(frontBeam) < sum(leftBeam) and sum(frontBeam) < sum(backBeam) and sum(frontBeam) < sum(rightBeam):
            self.closestBeam = "front"
        elif sum(leftBeam) < sum(frontBeam) and sum(leftBeam) < sum(backBeam) and sum(leftBeam) < sum(rightBeam):
            self.closestBeam = "left"
        elif sum(backBeam) < sum(frontBeam) and sum(backBeam) < sum(leftBeam) and sum(backBeam) < sum(rightBeam):
            self.closestBeam = "back"
        elif sum(rightBeam) < sum(frontBeam) and sum(rightBeam) < sum(leftBeam) and sum(rightBeam) < sum(backBeam):
            self.closestBeam = "right"
        else:
            self.closestBeam = 'none'

        # print("frontBeam: "+ str(sum(frontBeam)))
        # print("rightBeam: "+ str(sum(rightBeam)))
        # print("backBeam: "+ str(sum(backBeam)))
        # print("leftBeam: "+ str(sum(leftBeam)))

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

    lidarThread = Thread(target=robot.lidar_scan)
    lidarThread.daemon = True
    lidarThread.start()
    sleep(0.5)
    
    sensorGroundThread = Thread(target=robot.sensGround)
    sensorGroundThread.daemon = True
    sensorGroundThread.start()

    infraredCommSendThread = Thread(target=robot.sendInformation, args=([2])) ## args=(1) = seeker, args=(2) = avoider
    infraredCommSendThread.daemon = True
    infraredCommSendThread.start()

    infraredCommRecieveThread = Thread(target=robot.receiveInformation)
    infraredCommRecieveThread.daemon = True
    infraredCommRecieveThread.start()

    speed = 500
    robot.disable_sensor_leds()
    # Controller #
    while True:
        try:
            # FLEE
                robot.detection()
                #print("0: " + str(robot.sensorGroundValues[0]))
                #print("1: " + str(robot.sensorGroundValues[1]))
                
                ####### Basic behavior #######
                #print(robot.closestBeam)
                # TAPE AVOIDANCE
                if robot.sensorGroundValues[0] < 260 and robot.robotState != 'avoidLeft': 
                    robot.robotState = 'avoidLeft'
                elif robot.sensorGroundValues[1] < 260 and robot.robotState != 'avoidRight': 
                    robot.robotState = 'avoidRight'
                # # DETECT SAFE ZONE
                elif robot.lock == False and robot.sensorGroundValues[0] > 260 and robot.sensorGroundValues[0] < 1007 and robot.sensorGroundValues[1] > 260 and robot.sensorGroundValues[1] < 1007:
                    robot.lock = True
                    robot.robotState = 'moveIntoSafeZone'
                # LEAVE SAFE ZONE
                elif robot.rx[0] == 2 and robot.robotState == 'safeZone':
                    robot.drive(400, 400)
                    sleep(4)
                    robot.robotState = 'drive'
                    robot.lock = False
                # CAUGHT
                elif robot.robotState != 'caught' and robot.robotState != 'safeZone' and robot.rx[0] == 1:
                    robot.robotState = 'caught'

                print(robot.robotState)
                # State Behavior
                if robot.robotState == 'drive':
                    robot.LED('blue')
                    if robot.closestBeam == "left": 
                        robot.drive(speed, 0)
                    elif robot.closestBeam == "right": 
                        robot.drive(0, speed)
                    elif robot.closestBeam == "back": 
                        robot.drive(speed, speed)
                    elif robot.closestBeam == "front": 
                        robot.drive(-speed, speed)
                        sleep(0.5)
                        robot.drive(speed,speed)
                    elif robot.closestBeam == "none": 
                        robot.drive(0,0)
                elif robot.robotState == 'avoidLeft':
                    robot.drive(400, -400)
                    sleep(0.5)
                    robot.robotState = 'drive'
                elif robot.robotState == 'avoidRight':
                    robot.drive(-400, 400)
                    sleep(0.5)
                    robot.robotState = 'drive'
                elif robot.robotState == 'moveIntoSafeZone':
                    robot.LED("green")
                    robot.drive(500, 500)
                    sleep(0.75)
                    robot.stop()
                    robot.robotState = 'safeZone'
                elif robot.robotState == 'safeZone':
                    robot.LED("green")
                    robot.stop()
                    print(robot.rx[0])
                elif robot.robotState == 'caught':
                    robot.stop()
                    robot.LED("purple")
                    robot.lidar_stop()
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
    finally:
        robot.lidar_stop()
        robot.LED('off')
        print("Stopping robot")
        robot.exit_now = True
        robot.stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

