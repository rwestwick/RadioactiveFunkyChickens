#!/usr/bin/python
# SomewhereOverTheRainbow.pw
# http://piwars.org/2018-competition/challenges/somewhere-over-the-rainbow/

"""
This algorithm is designed to solve the Somewhere Over the Rainbow Pi Wars 4.0
challenge.
"""

# https://www.python.org/dev/peps/pep-0238/
# The future division statement, spelled "from __future__ import division", will change the / operator to mean true division
# throughout the module.
# This is needed for the row and column calculations for rectangle arrays to prevent rounding down to zero.
from __future__ import division

# Import needed libraries such as picamera OpenCV and NumPy
import logging
import time
import SetupConsoleLogger
import ServoController
import MotorController
import UltrasonicSensor
import GPIOLayout
import KeyboardCharacterReader
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

# Create a logger to both file and stdout
LOGGER = logging.getLogger("__name__")
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set initial constant values
FRONT_BUFFER_WARN = 35 # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 25 # Shortest distance to front (cm)
SIDE_BUFFER = 10 # Shortest distance to side (cm)
CORRECTION_TIME = 0.15 # Angle correction delay time in seconds
FORWARD_TIME = 0.1 # Angle correction delay time in seconds
TURN_DELAY = 0.65

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

# Initialise servos
SERVO_CONTROLLER = ServoController.ServoController()

# Initialize camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
camera = PiCamera() # Initialize camera
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT) # resolution defaults to dosplays resolution
# Can get framerates up to 60fps 640x480
camera.framerate = 10 # If not set then defaults to 30fps
camera.vflip = True
camera.hflip = True

# http://picamera.readthedocs.io/en/release-1.10/api_array.html
# class picamera.array.PiRGBArray(camera, size=None)[source]
# Produces a 3-dimensional RGB array from an RGB capture with the dimensions (rows, columns, plane)
# for example of size (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Allow the camera time to warmup
time.sleep(1)

def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    Method 1 - First choice
    Method 2 - Emergency backup
    """

    LOGGER.info("Minimal Maze")

    # Waiting for start of challenge
    LOGGER.info("To start 'Somewhere Over the Rainbow' press 'Space' key.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_PIN,
        GPIOLayout.SONAR_LEFT_TX_PIN)
    view_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN,
        GPIOLayout.SONAR_RIGHT_TX_PIN)
    view_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_PIN)

    LOGGER.info("Distance view_left at start " + format(view_left.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_right at start " + format(view_right.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_front at start " + format(view_front.measurement(), '.2f') + " cm")

    # Setting the camera to look ahead
    SERVO_CONTROLLER.start_servos()
    time.sleep(1) # Need to add a delay or echo does not work!
    SERVO_CONTROLLER.set_pan_servo(0)
    SERVO_CONTROLLER.set_tilt_servo(0) 

    # Start the challenge
    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    # Capture live video feed
    LOGGER.info("To close live video feeds press 'q' key.")

    # Capture frames from the camera
    # http://picamera.readthedocs.io/en/release-1.10/api_camera.html
    # capture_continuous(output, format=None, use_video_port=False, resize=None, splitter_port=0, burst=False, **options)
    # The format, use_video_port, splitter_port, resize, and options parameters are the same as in capture()

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array respresenting the image, then intialize the timestap
        # and occupied/unoccupied text
        image = frame.array

        trueColor = image

        # Define the colour boundaries in BGR
        # lower = [17, 15, 100] # Red
        # upper = [100, 100, 255] # Red
        lower = [100, 20, 20] # Blue
        upper = [255, 100, 100] # Blue

        # Create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        # find the colours within the specified boundaries and apply the mask
        mask = cv2.inRange(trueColor, lower, upper)
        output = cv2.bitwise_and(trueColor, trueColor, mask = mask)
        
        # show the frame(s) for test perpurposes
        cv2.imshow("ColourFrame", trueColor)
        cv2.imshow("Mask", mask)
        cv2.imshow("ColourThreshold", output)

        # Spin robot to work out position of each coloured marker

        # Point towards Red marker and go into quarter cirle zone

        # Point towards Blue marker and go into quarter cirle zone

        # Point towards Yellow marker and go into quarter cirle zone

        # Point towards Green marker and go into quarter cirle zone

        # http://picamera.readthedocs.io/en/release-1.10/api_array.html
        # Clear the stream in preperation for the next frame
        rawCapture.truncate(0)

        # Capture a key press. The function waits argument in ms for any keyboard event
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key was pressed break from the loop
        if key == ord("q"):
            break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Somewhere Over the Rainbow'.")
    finally:
        LOGGER.info("'Somewhere Over the Rainbow' Finished.")
        SERVO_CONTROLLER.stop_servos()
        ROBOTMOVE.cleanup()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        
        
