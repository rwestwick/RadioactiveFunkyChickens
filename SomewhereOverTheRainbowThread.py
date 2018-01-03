#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  SomewhereOverTheRainbowThread.py
# http://piwars.org/2018-competition/challenges/somewhere-over-the-rainbow/
"""
This algorithm is designed to solve the Somewhere Over the Rainbow Pi Wars 4.0
challenge.
"""

# https://www.python.org/dev/peps/pep-0238/
# The future division statement, spelled "from __future__ import division",
# will change the / operator to mean true division throughout the module.
# This is needed for the row and column calculations for rectangle arrays to
# prevent rounding down to zero.
from __future__ import division

# Import needed libraries such as picamera OpenCV and NumPy
import logging
import time
import math
import sys
import threading
import SetupConsoleLogger
import ServoController
import MotorController
import SpeedSettings
import UltrasonicSensor
import ColourBoundaries
import GPIOLayout
import KeyboardCharacterReader
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

# Global values
global camera
global processor

# Image stream processing thread
class StreamProcessor(threading.Thread):
    def __init__(self):
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.begin = 0


    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array, colour)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()


    # Image processing function
    def ProcessImage(self, bgr_mage, colour):

        # Blur the image
        bgr_image = cv2.medianBlur(bgr_image, 5)

        # Convert the image from 'BGR' to HSV colour space
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_RGB2HSV)
        
		# Select HSV colour range boundaries to detect marker
	    lower_hsv = ColourBoundaries.LOWER_HSV_ARRAY[colourArrayCntr]
	    upper_hsv = ColourBoundaries.UPPER_HSV_ARRAY[colourArrayCntr]
	    lower_red_lft_hsv = ColourBoundaries.LOWER_RED_LFT_HSV
	    upper_red_lft_hsv = ColourBoundaries.UPPER_RED_LFT_HSV
	
	    # Create HSV NumPy arrays from the boundaries
	    lower_hsv = np.array(lower_hsv, dtype="uint8")
	    upper_hsv = np.array(upper_hsv, dtype="uint8")
	    lower_red_lft_hsv = np.array(lower_red_lft_hsv, dtype="uint8")
	    upper_red_lft_hsv = np.array(upper_red_lft_hsv, dtype="uint8")
	    
        # Find the colours within the specified boundaries and apply the mask
	    mask_hsv = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
	    if colourArrayCntr == 0:
	        mask_hsv = mask_hsv + cv2.inRange(hsv_image, lower_red_lft_hsv,
	                                          upper_red_lft_hsv)
	
	    # Applying mask to BGR image gives true colours on display
	    output_hsv = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_hsv)
        

	    def find_marker_contour(self, mask, output_hsv):
			""" Compute the location of the marker contour."""
			
			# Calculate contours
		    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
		                                                cv2.CHAIN_APPROX_SIMPLE)
		    
	        # Calculate final number of largest area contours
		    if len(contours) == 0:
		        LOGGER.info("No contours found.")
		    elif len(contours) < NUM_OF_LARGEST_AREA_CONTOURS:
		        finalNumLargestAreaContours = len(contours)
		    else:
		        finalNumLargestAreaContours = NUM_OF_LARGEST_AREA_CONTOURS
    
    def contour_circularity(self, cnts):
	    """Compute the circularity of the contours in the array
	    The lower the value the less circular
	    Perfect circle has value of one"""
	
	    # Initialize the circularity array
	    circularityArray = []
	
	    # Calculate circularity for each contour in input array
	    for c in cnts:
	        AreaContour = cv2.contourArea(c)
	        Perimeter = cv2.arcLength(c, True)
	        if Perimeter != 0.0:
	            circularity = (4 * math.pi * AreaContour) / \
	                (math.pow(Perimeter, 2))
	        else:
	            circularity = 0
	
	        circularityArray.append(circularity)
	
	    # return an array of circularity values
	    return circularityArray


# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        global camera
        global processor
        print('Start the stream using the video port')
        camera.capture_sequence(self.TriggerStream(), format='bgr', use_video_port=True)
        print('Terminating camera processing...')
        processor.terminated = True
        processor.join()
        print('Processing terminated.')

    # Stream delegation loop
    def TriggerStream(self):
        global running
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()

# Initialise objects and constants

# Create a logger to both file and stdout
LOGGER = logging.getLogger("__name__")
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise servos
SERVO_CONTROLLER = ServoController.ServoController()

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN, GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN, GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

# Initialise camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
camera = PiCamera()  # Initialize camera
# Camera resolution defaults to the monitors resolution,
# but needs to be lower for speed of processing
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
camera.framerate = 10  # If not set then defaults to 30fps
camera.vflip = True  # Needed for mounting of camera on pan/tilt
camera.hflip = True  # Needed for mounting of camera on pan/tilt

processor = StreamProcessor()

# Set movement constant values
FRONT_BUFFER_WARN = 35  # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 20  # Shortest distance to front (cm)
SIDE_BUFFER = 10  # Shortest distance to side (cm)
CORRECTION_TIME = 0.15  # Angle correction delay time in seconds
FORWARD_TIME = 0.05  # Forward time iteration delay time in seconds
TURN_DELAY = 0.65  # Delay when turning in seconds
PAN_INTIAL = 20  # Initial pan angle in degrees
TILT_INTIAL = 20  # Initial tilt angle in degrees

# Set FONT for text on image/video
FONT = cv2.FONT_HERSHEY_COMPLEX_SMALL

# Image filtering constants
MED_FILTER_APRTRE_SIZE = 5  # Aperture size for median filter
# Initial number for down selecting large contours
# If number too small then will loose circular marker
NUM_OF_LARGEST_AREA_CONTOURS = 3
MIN_MARKER_AREA = 100  # Pixels - the final value to be decided from testing
MIN_MARKER_CIRCULARITY = 0.5  # Correct value to be decided


def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    Method 1 - First choice
    Method 2 - Emergency backup
    """

    LOGGER.info("Somewhere Over The Rainbow")

    # Start servos
    LOGGER.info("Start Pan/Tilt servos")
    SERVO_CONTROLLER.start_servos()
    time.sleep(1)

    # Set initial colour from COLOUR_NAME_ARRAY array position -
    # 'Red', 'Blue', 'Green', 'Yellow'
    colourArrayCntr = 0

    # Initialize photo capture
    imageNum = 1

    # Show commands and status
    LOGGER.info("Press 'q' to quit.")
    LOGGER.info("Press 'c' to change colour selector.")
    LOGGER.info("Press 'p' to take picture of current frame.")
    LOGGER.info("All key presses must be in a video frame window.")
    LOGGER.info("The colour selector is now " +
                ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr])

    # Waiting for start of challenge
    LOGGER.info("Press 'Space' in console to start.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(GPIOLayout.SONAR_LEFT_RX_PIN,
                                                  GPIOLayout.SONAR_LEFT_TX_PIN)
    view_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN, GPIOLayout.SONAR_RIGHT_TX_PIN)
    view_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_PIN)

    LOGGER.info("Distance view_left at start " +
                format(view_left.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_right at start " +
                format(view_right.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_front at start " +
                format(view_front.measurement(), '.2f') + " cm")

    # Start the challenge when space key presses
    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

	return 0

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Somewhere Over the Rainbow'.")
    finally:
        LOGGER.info("'Somewhere Over the Rainbow' Finished.")
        SERVO_CONTROLLER.stop_servos()
        cv2.destroyAllWindows()
        GPIO.cleanup()
