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
import threading
import SetupConsoleLogger
import ServoController
import DualMotorController
import SpeedSettings
import UltrasonicSensorThread
import ColourBoundaries
import GPIOLayout
import RPi.GPIO as GPIO
import cv2
import numpy as np
import picamera
import picamera.array

# Global values and their initial values
global camera
global processor
global running
global debug  # Used for processing time
global debug_show_input  # Used to show input image
global debug_show_output  # Used to show image output after processing
global debug_show_steering  # Used to show steering outputs
global max_processing_delay
global min_processing_delay
global colour_array_cntr

running = True
debug = True
debug_show_input = True
debug_show_output = True
debug_show_steering = True

# Set initial colour from COLOUR_NAME_ARRAY array position -
# 'Red', 'Blue', 'Green', 'Yellow'
colour_array_cntr = 0
max_processing_delay = 0  # Initialsed for delay calculations
min_processing_delay = 100  # Initialsed for delay calculations


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
        # The start() method starts a thread by calling the run method
        self.start()
        self.begin = 0

    def run(self):  # The run() method is the entry point for a thread
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for an image to be written to the stream
            # The wait() method takes an argument representing the 
            # number of seconds to wait for the event before timing out.
            if self.event.wait(1):
                try:
                    # Read the image and do some processing on it
                    self.stream.seek(0)
                    self.process_image(self.stream.array)
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()

    # Image processing function
    def process_image(self, image):
        global colour_array_cntr
        global max_processing_delay
        global min_processing_delay
        global debug
        global debug_show_input
        global debug_show_output

        # View the original image seen by the camera.
        if debug_show_input:
            cv2.imshow('Original BGR', image)
            # Capture a key press. The function waits argument in ms
            # for any keyboard event
            # For some reason image does not show without this!
            key_one = cv2.waitKey(1) & 0xFF 
        if debug:
            e1 = cv2.getTickCount()

        # Find chosen colour in image
        colour_filtered_output, colour_filtered_mask = self.find_HSV_colour(
            colour_array_cntr, image)

        # Find location of contour
        contourDetection, foundX, foundY, contour_marked_image = self.find_marker_contour(
            colour_filtered_mask, colour_filtered_output)

        # Calculate image processing overhead
        # https://docs.opencv.org/3.0.0/dc/d71/tutorial_py_optimization.html
        if debug:
            e2 = cv2.getTickCount()
            time = (e2 - e1) / cv2.getTickFrequency()
            if time > max_processing_delay:
                max_processing_delay = time
            elif time < min_processing_delay:
                min_processing_delay = time

        if debug_show_output:
            cv2.imshow('Filtered image with marker contour',
                       contour_marked_image)
        
            # Capture a key press. The function waits argument in ms
            # for any keyboard event
            # For some reason image does not show without this!
            key_two = cv2.waitKey(1) & 0xFF 
        
        if key_one == ord("c") or key_two == ord("c") :
            # Loop over integers 0 to 3
            colour_array_cntr = (colour_array_cntr + 1) % 4
            LOGGER.info("The colour selector is now " +
                    ColourBoundaries.COLOUR_NAME_ARRAY[colour_array_cntr])

        # Steer robot
        distance_to_front_wall = FRONT_SENSOR.read_data()
        self.set_speed_from_marker(contourDetection, 
                                    foundX, 
                                    distance_to_front_wall)

    def find_HSV_colour(self, colour_array_cntr, bgr_image):
        """Find chosen colours in video image using HSV
        Inputs:
        colour_array_cntr - Integer for selection of upper and lower colour bands
        bgrImage - BGR image from camera
        Outputs:
        Masked BGR image and its mask from colour detection
        Masked blurred BGR image and its mask from colour detection"""

        # Blur the image
        bgr_blur_image = cv2.medianBlur(bgr_image, MED_FILTER_APRTRE_SIZE)

        # Convert the image from 'BGR' to HSV colour space
        hsv_image = cv2.cvtColor(bgr_blur_image, cv2.COLOR_BGR2HSV)

        # Select HSV colour range boundaries to detect marker
        lower_hsv = ColourBoundaries.LOWER_HSV_ARRAY[colour_array_cntr]
        upper_hsv = ColourBoundaries.UPPER_HSV_ARRAY[colour_array_cntr]
        lower_red_lft_hsv = ColourBoundaries.LOWER_RED_LFT_HSV
        upper_red_lft_hsv = ColourBoundaries.UPPER_RED_LFT_HSV

        # Create HSV NumPy arrays from the boundaries
        lower_hsv = np.array(lower_hsv, dtype="uint8")
        upper_hsv = np.array(upper_hsv, dtype="uint8")
        lower_red_lft_hsv = np.array(lower_red_lft_hsv, dtype="uint8")
        upper_red_lft_hsv = np.array(upper_red_lft_hsv, dtype="uint8")

        # Find the colours within the specified boundaries and apply the mask
        mask_hsv = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
        if colour_array_cntr == 0:
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

            if debug:
                # Highlight the most circular contour in white
                cv2.drawContours(input_hsv, cntSortedByCirc[0], -1,
                                 (255, 255, 255), 3)

        return contourDetection, foundX, foundY, input_hsv

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
    def set_speed_from_marker(self, contourDetection, foundX,
                           distanceToFrontWall):
        """Calulates the speed of the motors."""

        if contourDetection:
            if distanceToFrontWall < FRONT_BUFFER_STOP:
                LOGGER.info('Too close!')
                ROBOTMOVE.stop()
                driveLeft = 0
                driveRight = 0
                speed = 0
            elif distanceToFrontWall < FRONT_BUFFER_WARN:
                LOGGER.info('Getting closer')
                speed = SpeedSettings.SPEED_SLOW
            else:
                speed = SpeedSettings.SPEED_FAST
            
            direction = (foundX - IMAGE_CENTRE_X) / IMAGE_CENTRE_X
            
            if debug_show_steering:
                TextStringSpeed = ' Distance: ' +  str(int(distanceToFrontWall)) + \
                                  ' Found X: ' +  str(foundX) + \
                                  ' Direction: ' + str(direction)
                LOGGER.info(TextStringSpeed)

            if direction > 0.0:
                # Turn to robot's right hand side
                LOGGER.info('Steer Right')
                driveLeft = speed
                driveRight = speed * (1.0 - direction)
            else:
                # Turn to robot's left hand side
                LOGGER.info('Steer Left')
                driveLeft = speed * (1.0 + direction)
                driveRight = speed
                ROBOTMOVE.turn_forward(driveLeft, driveRight)

            if debug_show_steering:
                TextStringSpeed = 'Left speed: ' + str(driveLeft) + \
                                  ' Right speed: ' + str(driveRight)
                LOGGER.info(TextStringSpeed)

        else:
            LOGGER.info('No marker')


# Image capture thread
class ImageCapture(threading.Thread):

    def __init__(self):
        super(ImageCapture, self).__init__()
        self.start(
        )  # The start() method starts a thread by calling the run method.

    def run(self):
        global camera
        global processor
        LOGGER.info('Start the stream using the video port')
        camera.capture_sequence(
            self.trigger_stream(), format='bgr', use_video_port=True)
        LOGGER.info('Terminating camera processing...')
        processor.terminated = True
        processor.join()  # The join() waits for threads to terminate
        LOGGER.info('Processing terminated.')

    # Stream delegation loop
    def trigger_stream(self):
        global running
        
        while running:
            if processor.event.is_set():
                time.sleep(0.01)
            else:
                yield processor.stream
                processor.event.set()

####################################
# Initialise objects and constants #
####################################

# Create a logger to both file and stdout
LOGGER = logging.getLogger("__name__")
SetupConsoleLogger.setup_console_logger(LOGGER, logging.DEBUG)

# Initialise servos
PAN_INTIAL = 0  # Initial pan angle in degrees
TILT_INTIAL = 20  # Initial tilt angle in degrees
SERVO_CONTROLLER = ServoController.ServoController()

# Initialise motors
ROBOTMOVE = DualMotorController.DualMotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_PIN,
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_PIN)
    
ROBOTMOVE.stop()

# Initialise camera
CAMERA_WIDTH = 320  # 320 x 240 used in PiBorg has been tested with 640 * 480
CAMERA_HEIGHT = 240
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
time.sleep(2)  # This is the value/line used in the PiBorg example
capture_thread = ImageCapture()

# Set movement constant values
FRONT_BUFFER_WARN = 35  # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 20  # Shortest distance to front (cm)
SIDE_BUFFER = 10  # Shortest distance to side (cm)
CORRECTION_TIME = 0.15  # Angle correction delay time in seconds
FORWARD_TIME = 0.05  # Forward time iteration delay time in seconds
TURN_DELAY = 0.65  # Delay when turning in seconds

# Set FONT for text on image/video
FONT = cv2.FONT_HERSHEY_COMPLEX_SMALL

# Image filtering constants
# Aperture size for median filter based on PiBorg numbers of 5 with 320 * 240
MED_FILTER_APRTRE_SIZE = 5  # Must be odd number
# Initial number for down selecting large contours
# If number too small then will loose circular marker
NUM_OF_LARGEST_AREA_CONTOURS = 3
MIN_MARKER_AREA = 100  # Pixels - the final value to be decided from testing
MIN_MARKER_CIRCULARITY = 0.5  # Correct value to be decided

# Initialise Ultrasonic Sensors
FRONT_SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
    1, None, GPIOLayout.SONAR_FRONT_TX_PIN, GPIOLayout.SONAR_FRONT_RX_PIN, 1)
FRONT_SENSOR.start()

RIGHT_SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
    1, None, GPIOLayout.SONAR_RIGHT_TX_PIN, GPIOLayout.SONAR_RIGHT_RX_PIN, 1)
RIGHT_SENSOR.start()

LEFT_SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
    1, None, GPIOLayout.SONAR_LEFT_TX_PIN, GPIOLayout.SONAR_LEFT_RX_PIN, 1)
LEFT_SENSOR.start()


def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    Method 1 - First choice
    Method 2 - Emergency backup
    """
    
    global running

    LOGGER.info("Somewhere Over The Rainbow")

    # Start servos
    LOGGER.info("Start Pan/Tilt servos")
    SERVO_CONTROLLER.start_servos()
    time.sleep(1)
    
    # Initialise pan/tilt direction
    SERVO_CONTROLLER.set_pan_servo(PAN_INTIAL)
    SERVO_CONTROLLER.set_tilt_servo(TILT_INTIAL)

    # Show commands and status
    LOGGER.info("CTRL^C to terminate program")
    LOGGER.info("Press 'c' to change colour")
    LOGGER.info("The colour selector starts as " +
                ColourBoundaries.COLOUR_NAME_ARRAY[colour_array_cntr])

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
        LOGGER.info('Read distance from front sensor ={0:0.2f} cm '.format(
            FRONT_SENSOR.read_data()))
        LOGGER.info("Image processing delays min and max " +
                    format(min_processing_delay, '.2f') + " " +
                    format(max_processing_delay, '.2f') + " sec")
        running = False  # What is the scope of this variable?
        capture_thread.join()
        processor.terminated = True
        processor.join()
        del camera  # Is this needed?
        FRONT_SENSOR.exit_now()
        FRONT_SENSOR.join()
        FRONT_SENSOR.__del__()
        RIGHT_SENSOR.exit_now()
        RIGHT_SENSOR.join()
        RIGHT_SENSOR.__del__()
        LEFT_SENSOR.exit_now()
        LEFT_SENSOR.join()
        LEFT_SENSOR.__del__()
        SERVO_CONTROLLER.stop_servos()
        ROBOTMOVE.cleanup()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        LOGGER.info("'Somewhere Over the Rainbow' Finished.")
