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
        self.sensorGroundValues = []
        self.rx = [0]
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
        # TEST IF WHILE TRUE IS NEEDED - DOCUMENTATION SAID IF ENABLED IT SENDS OUT EVERY 100 m/s
        while True: 
            self.aseba.SendEventName("prox.comm.tx", [number])

    def receiveInformation(self):
        while True: 
            self.rx = self.aseba.GetVariable("thymio-II", "prox.comm.rx")

    def LED(self): 
        self.aseba.SendEventName("leds.prox.h", [1, 2, 3, 4, 5, 6, 7, 8])

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

    while True: 
        try: 
            #print("0: " + str(robot.sensorGroundValues[0]))
            #print("1: " + str(robot.sensorGroundValues[1]))
            print(robot.rx[0])
            #robot.LED()
            ####### Basic behavior #######
            """
            if robot.sensorGroundValues[0] < 300:
                robot.drive(-100, 100)
            elif robot.sensorGroundValues[1] < 300:
                robot.drive(100, -100)
            else:
                robot.drive(200, 200)
            """
            sleep(1)
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
        exit_now = True
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")
