# this imports the camera

from picamera import PiCamera
from time import sleep
import numpy as np
import cv2 as cv2

#initialize

camera = PiCamera()

def initCamera():
    print("Camera test")
    camera.start_preview()
    sleep(1)

def capture():
    #we capture to openCV compatible format
    #you might want to increase resolution
    camera.resolution = (480, 640)
    camera.framerate = 24
    image = np.empty((640, 480, 3), dtype=np.uint8)
    camera.capture(image, 'bgr')
    sleep(0.1)
    # color detection
    hsvFrame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    
    # upper boundary RED color range values; Hue (160 - 180)
    lower1 = np.array([150,100,20])
    upper1 = np.array([189,255,255])
    full_mask = cv2.inRange(hsvFrame, lower1, upper1)
    red = cv2.bitwise_and(image, image, mask=full_mask)

    right = np.sum(red[:,0:213])
    mid = np.sum(red[:,213:426])
    left = np.sum(red[:,426:])

    maxVal = max([left,right,mid])

    if right == maxVal:
        print('right')
    elif mid == maxVal:
        print('mid')
    elif left == maxVal:
        print('left')
    else:
        print('No color detected')

def stopCamera():
    camera.stop_preview()
    
if __name__ == '__main__':
    # Init camera
    try:
        initCamera()
        while True:
            capture()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop camera
        stopCamera()
    