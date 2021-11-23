# this imports the camera

from picamera import PiCamera
import numpy as np
import cv2
from time import sleep

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
#initialize

camera = PiCamera()

def initCamera():
    print("Camera test")
    np.set_printoptions(threshold=np.inf)
    camera.start_preview()

def testCamera():
    camera.resolution = (320, 240)
    camera.framerate = 24
    sleep(2)
    output = np.empty((240, 320, 3), dtype=np.uint8)
    camera.capture(output, 'rgb')

def closeCamera():
    camera.stop_preview()

if __name__ == '__main__':
    initCamera()
    try:
        while True:
            testCamera()
    except KeyboardInterrupt:
        pass
    finally:
        closeCamera()

    # img = mpimg.imread('out.png')
    # imgplot = plt.imshow(img)
    # plt.show()