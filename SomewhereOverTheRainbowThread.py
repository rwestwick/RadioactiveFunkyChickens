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
import picamera
import picamera.array

# Global values and their initial values
global camera
global processor
global debug
global maxProcessingDelay
global minProcessingDelay
global colourArrayCntr

running = True
debug = True
# Set initial colour from COLOUR_NAME_ARRAY array position -
# 'Red', 'Blue', 'Green', 'Yellow'
colourArrayCntr = 0
maxProcessingDelay = 0
minProcessingDelay = 100

# Image stream processing thread
# For threading tutourials see
# https://www.tutorialspoint.com/python/python_multithreading.htm
# http://www.bogotobogo.com/python/Multithread/python_multithreading_Event_Objects_between_Threads.php
class StreamProcessor(threading.Thread):
    def __init__(self):
        super(StreamProcessor, self).__init__()
        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()  # The start() method starts a thread by calling the run method.
        self.begin = 0


    def run(self):  # The run() method is the entry point for a thread
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            # The wait() method takes an argument representing the number of 
            # seconds to wait for the event before timing out.
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.ProcessImage(self.stream.array)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()


    # Image processing function
    def ProcessImage(self, image):
        global colourArrayCntr
        global maxProcessingDelay
        global minProcessingDelay
        
        # View the original image seen by the camera.
        if debug:
            cv2.imshow('Original BGR', image)
            cv2.waitKey(1) # Is this needed?
            e1 = cv2.getTickCount()

        # Find chosen colour in image
        output_hsv, mask_hsv = self.find_HSV_colour(colourArrayCntr, image)

        # Find location of contour
        contourDetection, foundX, foundY = self.find_marker_contour(mask_hsv, output_hsv)

        # Calculate image processing overhead
        # https://docs.opencv.org/3.0.0/dc/d71/tutorial_py_optimization.html
        if debug:
            e2 = cv2.getTickCount()
            time = (e2 - e1)/ cv2.getTickFrequency()
            if time > maxProcessingDelay:
                maxProcessingDelay = time
            elif time < minProcessingDelay:
                minProcessingDelay = time

        # Steer robot
        # self.SetSpeedFromMarker(contourDetection, foundX, distanceToFrontWall)


    def find_HSV_colour(self, colourArrayCntr, bgr_image):
        """Find chosen colours in video image using HSV
        Inputs:
        colourArrayCntr - Integer for selection of upper and lower colour bands
        bgrImage - BGR image from camera
        Outputs:
        Masked BGR image and its mask from colour detection
        Masked blurred BGR image and its mask from colour detection"""
    
        # Blur the image
        bgr_blur_image = cv2.medianBlur(bgr_image, MED_FILTER_APRTRE_SIZE)

        # Convert the image from 'BGR' to HSV colour space
        hsv_image = cv2.cvtColor(bgr_blur_image, cv2.COLOR_RGB2HSV)
        
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

        return output_hsv, mask_hsv


    def find_marker_contour(self, mask, input_hsv):
        """ Compute the location of the marker contour."""
        
        # Calculate contours
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                    cv2.CHAIN_APPROX_SIMPLE)

        # Calculate final number of largest area contours
        if len(contours) == 0:
            LOGGER.info("No contours found.")
            finalNumLargestAreaContours = 0
            contourDetection = False
            foundX = None
            foundY = None
        elif len(contours) < NUM_OF_LARGEST_AREA_CONTOURS:
            finalNumLargestAreaContours = len(contours)
        else:
            finalNumLargestAreaContours = NUM_OF_LARGEST_AREA_CONTOURS

        # Sort for three largest contours by area
        cntSortedByArea = sorted(
            contours, key=cv2.contourArea,
            reverse=True)[:finalNumLargestAreaContours]

        # Calculate position of contour that still is greater than Min area
        cntWithMinArea = []
        for cntCounterArea in range(len(cntSortedByArea)):
            if cv2.contourArea(cntSortedByArea[cntCounterArea]) \
               >= MIN_MARKER_AREA:
                cntWithMinArea.append(cntSortedByArea[cntCounterArea])
            else:
                break

        # Check to see if any contours are found as circularity and
        # zip does not work without array
        if len(cntWithMinArea) == 0:
            contourDetection = False
            foundX = None
            foundY = None
        else:
            contourDetection = True
    
            # Calculate the largest contours' by area circularity value
            cntCircularity = self.contour_circularity(cntWithMinArea)
    
            # Sort contours in order of circularity
            (cntSortedByCirc, cntCircularity) = zip(*sorted(
                zip(cntWithMinArea, cntCircularity),
                key=lambda x: x[1],
                reverse=True))
    
            # Calculate centre of most circular contour
            foundX, foundY = self.contour_centre(cntSortedByCirc[0])

        return contourDetection, foundX, foundY


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


    def contour_centre(self, cntr):
        """ Compute the centre of the contour area"""
        
        M = cv2.moments(cntr)
    
        # Prevent division by 0
        if M["m00"] == 0:
            x, y, w, h = cv2.boundingRect(cntr)
            cX = x + (w / 2)
            cY = y + (h / 2)
        else:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
    
        # return the x and y co-ordinates of the centre of contours
        return cX, cY


    # Set the motor speeds from the marker position
    def SetSpeedFromMarker(self, contourDetection, foundX, distanceToFrontWall):
        """Calulates the speed of the motors."""
        
        if contourDetection:
            if distanceToFrontWall < FRONT_BUFFER_STOP:
                LOGGER.info('Too close!')
                ROBOTMOVE.stop()
                driveLeft = 0
                driveRight = 0
            elif distanceToFrontWall < FRONT_BUFFER_WARN:
                LOGGER.info('Getting closer')
                speed = SpeedSettings.SPEED_SLOW
                driveLeft
            else:
                speed = SpeedSettings.SPEED_FAST

            direction = (foundX - IMAGE_CENTRE_X) / IMAGE_CENTRE_X
            if direction < 0.0:
                # Turn right
                LOGGER.info('Steer Right')
                driveLeft = speed
                driveRight = speed * (1.0 + direction)
            else:
                # Turn left
                LOGGER.info('Steer Left')
                driveLeft = speed * (1.0 - direction)
                driveRight = speed
                ROBOTMOVE.turn_forward(driveLeft, driveRight)
                
            TextStringSpeed = 'Left speed: ' + str(driveLeft) + \
                ' Right speed: ' + str(driveRight)
            LOGGER.info(TextStringSpeed)
            
        else:
            LOGGER.info('No marker')


# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start() # The start() method starts a thread by calling the run method.

    def run(self):
        global camera
        global processor
        LOGGER.info('Start the stream using the video port')
        camera.capture_sequence(self.TriggerStream(), format='bgr', use_video_port=True)
        LOGGER.info('Terminating camera processing...')
        processor.terminated = True
        processor.join()  # The join() waits for threads to terminate
        LOGGER.info('Processing terminated.')

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
IMAGE_CENTRE_X = CAMERA_WIDTH / 2.0
IMAGE_CENTRE_y = CAMERA_HEIGHT / 2.0

camera = picamera.PiCamera()
# Camera resolution defaults to the monitors resolution,
# but needs to be lower for speed of processing
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
camera.framerate = 10  # If not set then defaults to 30fps
camera.vflip = True  # Needed for mounting of camera on pan/tilt
camera.hflip = True  # Needed for mounting of camera on pan/tilt

# Start thread objects
LOGGER.info('Setup the stream processing thread')
processor = StreamProcessor()

LOGGER.info('Wait ...')
# Allow the camera time to warm-up
time.sleep(2) # This is the value/line used in the PiBorg example
captureThread = ImageCapture()

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

    # Initialize photo capture
    imageNum = 1

    # Show commands and status
    LOGGER.info("CTRL^C to terminate program")
    LOGGER.info("The colour selector is now " +
                ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr])

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

    # Loop indefinitely until we are no longer running
    while running:
        # Wait for the interval period
        time.sleep(1.0)



if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Somewhere Over the Rainbow'.")
    finally:
        running = False
        captureThread.join()
        processor.terminated = True
        processor.join()
        del camera  # Is this needed?
        SERVO_CONTROLLER.stop_servos()
        ROBOTMOVE.cleanup()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        LOGGER.info("Image processing delays min and max " +
                format(minProcessingDelay, '.2f') + " " +
                format(maxProcessingDelay, '.2f') + " sec")
        LOGGER.info("'Somewhere Over the Rainbow' Finished.")
