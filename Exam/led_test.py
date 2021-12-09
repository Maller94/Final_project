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

    def drive(self, left_wheel_speed, right_wheel_speed):        
        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

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
    try: 
        sleep(1)
        robot.LED("blue")
    except:
        "setting up"
        sleep(1)
    
    
#------------------- Main loop end ------------------------

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