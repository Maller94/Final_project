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
        
    ############## LIDAR ###############################
    def lidar_scan(self):
        for scan in self.lidar.iter_scans():
            # if(self.exit_now):
            #    return
            for (_, angle, distance) in scan:
                if distance > 400:
                    self.scan_data[min([359, floor(angle)])] = 400
                else:
                    self.scan_data[min([359, floor(angle)])] = distance

    def lidar_stop(self):
        self.lidar.stop()
        self.lidar.disconnect()

    def detection(self):
        frontBeam = self.scan_data[135:224] 
        rightBeam = self.scan_data[225:314] 
        backBeam = self.scan_data[315:] + self.scan_data[0:44]
        leftBeam = self.scan_data[45:134] 
        
        if sum(frontBeam) < sum(leftBeam) and sum(frontBeam) < sum(backBeam) and sum(frontBeam) < sum(rightBeam):
            self.closestBeam = "front"
        elif sum(leftBeam) < sum(frontBeam) and sum(leftBeam) < sum(backBeam) and sum(leftBeam) < sum(rightBeam):
            self.closestBeam = "left"
        elif sum(backBeam) < sum(frontBeam) and sum(backBeam) < sum(leftBeam) and sum(backBeam) < sum(rightBeam):
            self.closestBeam = "back"
        elif sum(rightBeam) < sum(frontBeam) and sum(rightBeam) < sum(leftBeam) and sum(rightBeam) < sum(backBeam):
            self.closestBeam = "right"
        #print("frontBeam: "+ str(frontBeam))
        #print("rightBeam: "+ str(rightBeam))
        #print("backBeam: "+ str(backBeam))
        #print("leftBeam: "+ str(leftBeam))

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
    for i in range(10): 
        try: 
            lidarThread = Thread(target=robot.lidar_scan)
            lidarThread.daemon = True
            lidarThread.start()

            sensorGroundThread = Thread(target=robot.sensGround)
            sensorGroundThread.daemon = True
            sensorGroundThread.start()

            infraredCommSendThread = Thread(target=robot.sendInformation, args=([2])) ## args=(1) = seeker, args=(2) = avoider
            infraredCommSendThread.daemon = True
            infraredCommSendThread.start()

            infraredCommRecieveThread = Thread(target=robot.receiveInformation)
            infraredCommRecieveThread.daemon = True
            infraredCommRecieveThread.start()
            break
        except: 
            print("setting up threads")
            sleep(1)


    # Controller #
    while True:
        try:
            # FLEE
            #if robot.sensorGroundValues[0] > 998 and robot.sensorGroundValues[1] > 1003: 
                robot.LED('blue') 
                robot.detection()
                print(robot.closestBeam)
                #print("0: " + str(robot.sensorGroundValues[0]))
                #print("1: " + str(robot.sensorGroundValues[1]))
                
                ####### Basic behavior #######
                if robot.closestBeam == "left": 
                    robot.drive(100, 0)
                elif robot.closestBeam == "right": 
                    robot.drive(0, 100)
                elif robot.closestBeam == "back": 
                    robot.drive(100, 100)
                elif robot.closestBeam == "front": 
                    robot.drive(-100, 100)
                    sleep(0.5)
                    robot.drive(100,100)
                else: 
                    robot.drive(100,100)
                
                sleep(1)
            # # AVOIDANCE
            # elif robot.sensorGroundValues[0] < 230: 
            #     robot.drive(400, -400)
            #     sleep(0.5) 
            # elif robot.sensorGroundValues[1] < 229: 
            #     robot.drive(-400, 400)
            #     sleep(0.5)
            # # DETECT SAFE ZONE
            # elif robot.sensorGroundValues[0] > 230 and robot.sensorGroundValues[0] < 1006 and robot.sensorGroundValues[1] > 229 and robot.sensorGroundValues[1] < 1010:
            #     robot.LED("green")
            #     robot.drive(300, 300)
            #     sleep(0.5)
            #     robot.stop()
            # # LEAVE SAFE ZONE
            # elif robot.rx[0] == 2: 
            #     robot.LED("blue")
            #     robot.drive(400, 400)
            #     sleep(1.5)
            # # CAUGHT
            # elif robot.rx[0] == 1: 
            #     robot.stop()
            #     robot.LED("purple")
            #     robot.lidar_stop()
            #     break
    
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
        robot.lidar_stop()
        robot.stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
    finally:
        robot.lidar_stop()
        robot.LED('off')
        print("Stopping robot")
        robot.stop()
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")

