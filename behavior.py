#!/usr/bin/python3
import os
# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import matplotlib.pyplot as plt
from time import sleep
import dbus
import dbus.mainloop.glib
from threading import Thread

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
            prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
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



#------------------ Main loop here -------------------------

def main():
    ####### Threads #######
    sensorHorizontalthread = Thread(target=robot.sensHorizontal)
    sensorHorizontalthread.daemon = True
    sensorHorizontalthread.start()


    ####### Avoidance behavior #######
    while True:
        try: 
            if robot.sensorHorizontalValues[0] > 2500: 
                robot.drive(200, 0)
            elif robot.sensorHorizontalValues[4] > 2500:
                robot.drive(0, 200)
            else: 
                robot.drive(200, 200)
        except: 
            sleep(1)
            print("Setting up...")

#------------------- Main loop end ------------------------

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
