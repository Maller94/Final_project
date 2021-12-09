#!/usr/bin/python3
# this imports the camera

from adafruit_rplidar import RPLidar
from random import random
from threading import Thread
from math import cos, sin, pi, floor
import dbus.mainloop.glib
import dbus
from time import sleep
import os
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")


class Thymio:
    ############## INIT ###############################
    def __init__(self):
        self.aseba = self.setup()
        # Setup the RPLidar
        self.PORT_NAME = '/dev/ttyUSB0'
        self.lidar = RPLidar(None, self.PORT_NAME)
        # This is where we store the lidar readings
        self.scan_data = [0]*360
        self.closestBeam = "none"

    ############## DRIVER ###############################
    # max speed is 500 = 20cm/s
    def drive(self, left_wheel, right_wheel):
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

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
        print("frontBeam: "+ str(frontBeam))
        print("rightBeam: "+ str(rightBeam))
        print("backBeam: "+ str(backBeam))
        print("leftBeam: "+ str(leftBeam))

    

############## Bus and aseba setup ######################################

    def setup(self):
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

def main():

    # Lidar Thread
    lidar_thread = Thread(target=robot.lidar_scan)
    lidar_thread.daemon = True
    lidar_thread.start()

    try: 
        while True: 
            robot.detection()
            print(robot.closestBeam)
            sleep(1)
    except:
        "setting up"
        sleep(1)    


# ------------------- Main loop end ------------------------
if __name__ == '__main__':
    robot = Thymio()
    try:
        main()
    except KeyboardInterrupt:
        print("Stopping robot")
        exit_now = True
        sleep(1)
        os.system("pkill -n asebamedulla")
        print("asebamodulla killed")