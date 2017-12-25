#!/usr/bin/python
"""
Provides ability to use the Raspberry Pi Camera to detect the line
to follow for the line follwowing challenge in PiWars 2017
"""

# https://www.python.org/dev/peps/pep-0238/
# The future division statement, spelled "from __future__ import division", 
# will change the / operator to mean true division
# throughout the module.
# This is needed for the row and column calculations for rectangle arrays
# to prevent rounding down to zero.
from __future__ import division

# Open relvant modules
import logging
import ServoController
import SetupConsoleLogger
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Set constants
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
ROW_LENGTH = 10  # Number of rectangles per row for black/white analysis
COL_LENGTH = 10  # Number of rectangles per column for black/white analysis

# Create a logger to both file and stdout
MODULE_LOGGER = logging.getLogger("__main__.LineFollowerCamera")


class LineFollowerCamera(object):
    """
    Provides ability to detect the angle of the line in front of the
    robot.
    """

    def __init__(self):
        """
        Initialises the class
        """
        MODULE_LOGGER.info("Setting up LineFollowerCamera Module")

        # Set camera to point in correct direction
        self.SERVO_CONTROLLER = ServoController.ServoController()
        self.SERVO_CONTROLLER.start_servos()
        self.SERVO_CONTROLLER.set_pan_servo(-45)
        self.SERVO_CONTROLLER.set_tilt_servo(0)

        # Initise PiCamera
        self.camera = PiCamera()  # Initialize camera
        # resolution defaults to dosplays resolution
        self.camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
        # Can get framerates up to 60fps 640x480
        self.camera.framerate = 10  # If not set then defaults to 30fps
        self.camera.vflip = True

        # http://picamera.readthedocs.io/en/release-1.10/api_array.html
        # class picamera.array.PiRGBArray(camera, size=None)[source]
        # Produces a 3-dimensional RGB array from an RGB capture with the 
        # dimensions (rows, columns, plane)
        # for example of size (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
        self.rawCapture = PiRGBArray(
            self.camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

        # Allow the camera time to warmup
        time.sleep(0.1)

        # Initialize rowValues array to do testing such that they are all
        # initialised to be white (255)
        self.MeanValues = np.ones([ROW_LENGTH, COL_LENGTH]) * 255

    def getLinePosition(self):
        """
        Get the position of the line in the camera view
        """
        for self.frame in self.camera.capture_continuous(
                self.rawCapture, format="rgb", use_video_port=True):
            # grab the raw NumPy array respresenting the image, 
            # then intialize the timestap
            # and occupied/unoccupied text
            self.image = self.frame.array

            # Capture a key press. The function waits argument in ms for any
            # keyboard event
            self.key = cv2.waitKey(1) & 0xFF

            # if the 'q' key was pressed break from the loop
            if self.key == ord("q"):
                break

        return 5

    def cleanUp(self):
        """
        Stops the servos by calling the function
        """
        self.SERVO_CONTROLLER.stop_servos()


if __name__ == "__main__":
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)

        # Initialize camera
        AngleCamera = LineFollowerCamera()

        # Get Line Positions
        position = AngleCamera.getLinePosition()
        print("The position is: ", position)

    except KeyboardInterrupt:
        pass
    finally:
        AngleCamera.cleanUp()
        MODULE_LOGGER.info("End of test")
