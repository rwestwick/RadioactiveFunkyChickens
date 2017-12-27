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
import SetupConsoleLogger
import KeyboardCharacterReader

import ServoController
import MotorController
import GPIOLayout

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
THRESHOLD = 50  # Used to classify gray as black or white. Lower number makes
# darker shades of grey go to white

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)
LOGGER.info("Setting up LineFollowerCamera Algorithm")

# Set camera to point in correct direction
SERVO_CONTROLLER = ServoController.ServoController()
SERVO_CONTROLLER.start_servos()
SERVO_CONTROLLER.set_pan_servo(-45)  # I thought this was meant to be tilt!?
SERVO_CONTROLLER.set_tilt_servo(0)

# Initise PiCamera
camera = PiCamera()  # Initialize camera
# resolution defaults to dosplays resolution
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
# Can get framerates up to 60fps 640x480
camera.framerate = 10  # If not set then defaults to 30fps
camera.vflip = True

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN, GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN, GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

# http://picamera.readthedocs.io/en/release-1.10/api_array.html
# class picamera.array.PiRGBArray(camera, size=None)[source]
# Produces a 3-dimensional RGB array from an RGB capture with the
# dimensions (rows, columns, plane)
# for example of size (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Allow the camera time to warmup
time.sleep(0.1)

# Initialize rowValues array to do testing such that they are all
# initialised to be white (255)
MeanValues = np.ones([ROW_LENGTH, COL_LENGTH]) * 255

# Performs the line following Camera algorithm
LOGGER.info("Line Follower With Move")

# Waiting for start of race
LOGGER.info("To start line following press 'Space' key.")

while True:
    keyp = KeyboardCharacterReader.readkey()
    if keyp == ' ':
        LOGGER.info("Go")
        break

# Capture frames from the camera
# http://picamera.readthedocs.io/en/release-1.10/api_camera.html
# capture_continuous(output, format=None, use_video_port=False, resize=None,
# splitter_port=0, burst=False, **options)
# The format, use_video_port, splitter_port, resize, and options
# parameters are the same as in capture()

for frame in camera.capture_continuous(
        rawCapture, format="rgb", use_video_port=True):
    # grab the raw NumPy array representing the image,
    # then intialize the timestamp and occupied/unoccupied text
    image = frame.array

    # Convert to gray scale
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # Simple thresholding of gray sclae image
    # http://www.pyimagesearch.com/2014/09/08/thresholding-simple-image-segmentation-using-opencv/
    # (T, threshImage) = cv2.threshold(src, thresh, maxval, type)
    # src - source image. This image should be grayscale.
    # thresh - is the threshold value which is used to classify the pixel
    # intensities in the grayscale image.
    # maxval - is the pixel value used if any given pixel in the image passes
    # the thresh  test.
    # type - the thresholding method to be used. The type  value can be any of:
    # cv2.THRESH_BINARY
    # cv2.THRESH_BINARY_INV
    # cv2.THRESH_TRUNC
    # cv2.THRESH_TOZERO
    # cv2.THRESH_TOZERO_INV

    ret, threshImg = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY)

    # show the frames - used for testing and not essential for algorithm
    # and will not work will over SSH/X
    # cv2.imshow("ColourFrame", image)
    # cv2.imshow("GrayFrame", gray)
    cv2.imshow("ThresholdFrame", threshImg)

    # Capture a key press. The function waits argument in ms for any keyboard
    # event
    key = cv2.waitKey(1) & 0xFF

    # Capture number of white/black pixels in ROW_LENGTH rectanges along
    # lower row of threshold frame
    # N.B. May want to make this for several rows to track the line further
    # in the horizon and allow for sharp 90deg turns.

    # Loop over all rows
    for j in range(COL_LENGTH):

        # Loop over all columns
        for i in range(ROW_LENGTH):

            # Image region of interest (ROI)
            startRow = int((j / COL_LENGTH) * CAMERA_HEIGHT)
            stopRow = int(((j + 1) / COL_LENGTH) * CAMERA_HEIGHT) - 1.0
            startCol = int((i / ROW_LENGTH) * CAMERA_WIDTH)
            stopCol = int(((i + 1) / ROW_LENGTH) * CAMERA_WIDTH) - 1.0

            square = threshImg[startRow:stopRow, startCol:stopCol]

            # Mean of all the values in rectangular "square" array
            MeanValues[j, i] = int(np.mean(square))

    # Find index of first minimum mean value per row
    # N.B. Black = 0, White = 255
    # As it is the first then if there are two fully black rectangles
    # this could lead to errors
    # print("The mean values array: ", MeanValues)
    # smallSquareTop = np.argmin(MeanValues[0, 0:(ROW_LENGTH-1)])
    # LOGGER.info("The rectangle with the most black pixels in top row is: ",
    #             str(smallSquareTop))
    smallSquareBottom = np.argmin(
        MeanValues[(COL_LENGTH - 1), 0:(ROW_LENGTH - 1)])
    LOGGER.info("The rectangle with the most black pixels in bottom row is: " +
                str(smallSquareBottom))

    # http://picamera.readthedocs.io/en/release-1.10/api_array.html
    # Clear the stream in preperation for the next frame
    rawCapture.truncate(0)

    # Straight On
    if smallSquareBottom == int(ROW_LENGTH / 2) or smallSquareBottom == int(
            (ROW_LENGTH / 2) - 1):  # Needed for even numbers to get middle two
        LOGGER.info(
            "Go Forward. Index of mean row value: " + str(smallSquareBottom))
        ROBOTMOVE.forward(MotorController.SPEED_MEDIUM)

    # Go Right
    elif smallSquareBottom > (int(ROW_LENGTH * 3 / 4) - 1):
        LOGGER.info("Turn Left fast. Index of mean row value: " +
                    str(smallSquareBottom))
        ROBOTMOVE.one_wheel_left(MotorController.SPEED_FAST)
    elif smallSquareBottom > int(ROW_LENGTH / 2) and smallSquareBottom <= (
            int(ROW_LENGTH * 3 / 4) - 1):
        LOGGER.info("Turn Left medium. Index of mean row value: " +
                    str(smallSquareBottom))
        ROBOTMOVE.one_wheel_left(MotorController.SPEED_MEDIUM)

    # Go Left
    elif smallSquareBottom < int(ROW_LENGTH / 4):
        LOGGER.info("Turn Right fast. Index of mean row value: " +
                    str(smallSquareBottom))
        ROBOTMOVE.one_wheel_right(MotorController.SPEED_FAST)
    elif smallSquareBottom < int(ROW_LENGTH / 2) and smallSquareBottom >= int(
            ROW_LENGTH / 4):
        LOGGER.info("Turn Right medium. Index of mean row value: " +
                    str(smallSquareBottom))
        ROBOTMOVE.one_wheel_right(MotorController.SPEED_MEDIUM)

    # Something is not quite right!?
    else:
        LOGGER.error("Some other state! Index of mean row value: " +
                     str(smallSquareBottom))
        ROBOTMOVE.stop()

    # if the 'q' key was pressed break from the loop
    if key == ord("q"):
        break

# simply destroys all windows created - used for testing and not essential
# for algorithm
# Can use cv2.destroyWindow(frameName) to destroy a specific window
cv2.destroyAllWindows()

# Close servos and motors
SERVO_CONTROLLER.stop_servos()
ROBOTMOVE.stop()
ROBOTMOVE.cleanup()

LOGGER.info("End of line following")
