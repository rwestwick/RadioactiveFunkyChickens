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
import logging
import time
import math
import cv2
import numpy as np
import SetupConsoleLogger
import ColourBoundaries
import GPIOLayout
import SpeedSettings
import ServoController
import DualMotorController
import UltrasonicSensorThread
import CameraThread

# Global values and their initial values
global debug  # Used for processing time
global debug_show_input  # Used to show input image
global debug_show_output  # Used to show image output after processing
global debug_show_steering  # Used to show steering outputs
global debug_show_tilt  # Used to show stilt value

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER, logging.DEBUG)

debug = True
debug_show_input = True
debug_show_output = True
debug_show_steering = True
debug_show_tilt = True


# Image stream processing thread
# For threading tutourials see
# https://www.tutorialspoint.com/python/python_multithreading.htm
# http://www.bogotobogo.com/python/Multithread/python_multithreading_Event_Objects_between_Threads.php
class Processor(object):
    """
    """

    PAN_INTIAL = 0  # Initial pan angle in degrees
    TILT_INTIAL = 20  # Initial tilt angle in degrees
    TILT_CHANGE = 2  # Change in tilt angle in degrees to keep marker centred
    MIN_TILT = -40  # Minimum tilt angle in degrees
    MAX_TILT = 40  # Maximum tilt angle in degrees
    STEERING_RATE = 4.0  # Changes the speed at which it turns towards ball
    MAX_SPEED = SpeedSettings.SPEED_FAST
    MIN_SPEED = 0
    # Set movement constant values
    FRONT_BUFFER_WARN = 35  # Distance to corner for when to slow (cm)
    FRONT_BUFFER_STOP = 20  # Shortest distance to corner (cm)
    CENTRE_BUFFER_WARN = 65  # Distance from corner for when to slow (cm)
    CENTRE_BUFFER_STOP = 80  # Shortest distance from corner (cm)
    # ~86.2 cm from corner if perfect centre
    SPIN_TIME = 1.0  # Spin time in seconds
    # Image filtering constants
    # Aperture size for median filter based on PiBorg numbers of 5 with 320 *
    # 240
    MED_FILTER_APRTRE_SIZE = 5  # Must be odd number
    # Initial number for down selecting large contours
    # If number too small then will loose circular marker
    NUM_OF_LARGEST_AREA_CONTOURS = 3
    MIN_MARKER_AREA = 100  # Pixels - the final value to be decided from testing

    def __init__(self):
        """
        Constructor
        """
        LOGGER.debug("StreamProcessor constructor called")

        self.width = 320
        self.height = 240
        self.reached_marker = False
        self.servo_controller = ServoController.ServoController()
        self.servo_controller.start_servos()
        self.pan_angle = self.PAN_INTIAL
        self.tilt_angle = self.TILT_INTIAL
        self.servo_controller.set_pan_servo(self.pan_angle)
        self.servo_controller.set_tilt_servo(self.tilt_angle)
        time.sleep(1)
        self.colour_array_cntr = 0
        LOGGER.info("The colour selector starts as " +
                    ColourBoundaries.COLOUR_NAME_ARRAY[self.colour_array_cntr])

        # Initialise Ultrasonic Sensors
        self.FRONT_SENSOR = UltrasonicSensorThread.UltrasonicSensorThread(
            1, None, GPIOLayout.SONAR_FRONT_TX_GPIO,
            GPIOLayout.SONAR_FRONT_RX_GPIO, 1)
        self.FRONT_SENSOR.start()

        self.ROBOTMOVE = DualMotorController.DualMotorController(
            GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)

    def __del__(self):
        """
        Destructor
        """
        self.cleanup()

    def cleanup(self):
        LOGGER.debug("starting cleanup")
        self.ROBOTMOVE.stop()
        self.ROBOTMOVE.cleanup()
        self.FRONT_SENSOR.exit_now()
        self.FRONT_SENSOR.join()
        self.servo_controller.stop_servos()

    def image_process_entry(self, bgr_image, width, height):
        """
        The main image processing function
        """
        global debug
        global debug_show_input
        global debug_show_output
        self.width = width
        self.height = height

        key_one = cv2.waitKey(1) & 0xFF

        # Find chosen colour in image
        colour_filtered_output, colour_filtered_mask = self.find_HSV_colour(
            bgr_image)

        # Find location of contour
        contour_detection, found_x, found_y, contour_marked_image = 
            self.find_marker_contour(
                colour_filtered_mask, colour_filtered_output)

        if debug_show_output:
            cv2.imshow('Filtered image with marker contour',
                       contour_marked_image)

            # Capture a key press. The function waits argument in ms
            # for any keyboard event
            # For some reason image does not show without this!
            key_two = cv2.waitKey(1) & 0xFF

        if key_one == ord("c") or key_two == ord("c"):
            # Loop over integers 0 to 3
            self.colour_array_cntr = (self.colour_array_cntr + 1) % 4
            LOGGER.info("The colour selector is now " + ColourBoundaries.
                        COLOUR_NAME_ARRAY[self.colour_array_cntr])

        if contour_detection:
            # Adjust tilt of Pi Camera
            self.set_camera_tilt_from_marker(found_y)

            # Steer robot
            distance_to_front_wall = self.FRONT_SENSOR.read_data()
            if not self.reached_marker:
                self.set_speed_from_marker(contour_detection, found_x,
                                           distance_to_front_wall)
            else:
                self.set_speed_to_centre(contour_detection, found_x,
                                         distance_to_front_wall)
        else:
            LOGGER.info("Spin right to find " + ColourBoundaries.
                        COLOUR_NAME_ARRAY[self.colour_array_cntr] + " marker.")
            self.spin_to_find_colour()

    def find_HSV_colour(self, bgr_image):
        """
        Find chosen colours in video image using HSV
        Inputs:
        bgrImage - BGR image from camera
        Outputs:
        Masked BGR image and its mask from colour detection
        Masked blurred BGR image and its mask from colour detection
        """
        # Blur the image
        bgr_blur_image = cv2.medianBlur(bgr_image, self.MED_FILTER_APRTRE_SIZE)

        # Convert the image from 'BGR' to HSV colour space
        hsv_image = cv2.cvtColor(bgr_blur_image, cv2.COLOR_BGR2HSV)

        # Select HSV colour range boundaries to detect marker
        lower_hsv = ColourBoundaries.LOWER_HSV_ARRAY[self.colour_array_cntr]
        upper_hsv = ColourBoundaries.UPPER_HSV_ARRAY[self.colour_array_cntr]
        lower_red_lft_hsv = ColourBoundaries.LOWER_RED_LFT_HSV
        upper_red_lft_hsv = ColourBoundaries.UPPER_RED_LFT_HSV

        # Create HSV NumPy arrays from the boundaries
        lower_hsv = np.array(lower_hsv, dtype="uint8")
        upper_hsv = np.array(upper_hsv, dtype="uint8")
        lower_red_lft_hsv = np.array(lower_red_lft_hsv, dtype="uint8")
        upper_red_lft_hsv = np.array(upper_red_lft_hsv, dtype="uint8")

        # Find the colours within the specified boundaries and apply the mask
        mask_hsv = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
        if self.colour_array_cntr == 0:
            mask_hsv = mask_hsv + cv2.inRange(hsv_image, lower_red_lft_hsv,
                                              upper_red_lft_hsv)

        # Applying mask to BGR image gives true colours on display
        output_hsv = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_hsv)

        return output_hsv, mask_hsv

    def find_marker_contour(self, mask, input_hsv):
        """
        Compute the location of the marker contour.
        """
        # Calculate contours
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                    cv2.CHAIN_APPROX_SIMPLE)

        # Calculate final number of largest area contours
        if len(contours) == 0:
            LOGGER.info("No contours found.")
            finalNumLargestAreaContours = 0
            contourDetection = False
            found_x = None
            found_y = None
        elif len(contours) < self.NUM_OF_LARGEST_AREA_CONTOURS:
            finalNumLargestAreaContours = len(contours)
        else:
            finalNumLargestAreaContours = self.NUM_OF_LARGEST_AREA_CONTOURS

        # Sort for three largest contours by area
        cntSortedByArea = sorted(
            contours, key=cv2.contourArea,
            reverse=True)[:finalNumLargestAreaContours]

        # Calculate position of contour that still is greater than Min area
        cntWithMinArea = []
        for cntCounterArea in range(len(cntSortedByArea)):
            if cv2.contourArea(cntSortedByArea[cntCounterArea]) \
               >= self.MIN_MARKER_AREA:
                cntWithMinArea.append(cntSortedByArea[cntCounterArea])
            else:
                break

        # Check to see if any contours are found as circularity and
        # zip does not work without array
        if len(cntWithMinArea) == 0:
            contourDetection = False
            found_x = None
            found_y = None
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
            found_x, found_y = self.contour_centre(cntSortedByCirc[0])

            if debug:
                # Highlight the most circular contour in white
                cv2.drawContours(input_hsv, cntSortedByCirc[0], -1,
                                 (255, 255, 255), 3)

        return contourDetection, found_x, found_y, input_hsv

    def contour_circularity(self, cnts):
        """
        Compute the circularity of the contours in the array
        The lower the value the less circular
        Perfect circle has value of one
        """
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
        """
        Compute the centre of the contour area
        """
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

    def set_camera_tilt_from_marker(self, found_y):
        """
        Set tilt ange of the Pi Camera.
        """
        #  Up/down direction - Value between -1.0 to +1.0
        # -1.0 is top, +1.0 is bottom, 0.0 is centre
        camera_direction = (found_y - (self.height / 2)) / (self.height / 2.0)

        if camera_direction < -0.5:
            self.tilt_angle = self.tilt_angle + self.TILT_CHANGE
        elif camera_direction > 0.5:
            self.tilt_angle = self.tilt_angle - self.TILT_CHANGE
        self.tilt_angle = sorted(
            [self.MIN_TILT, self.tilt_angle, self.MAX_TILT])[1]
        self.servo_controller.set_tilt_servo(self.tilt_angle)

        if debug_show_tilt:
            TextStringSpeed = 'Found Y: ' + str(found_y) + \
                              ' Tilt angle: ' + str(self.tilt_angle)
            LOGGER.info(TextStringSpeed)

    def set_speed_from_marker(self, contourDetection, found_x,
                              distanceToFrontWall):
        """
        Set the motor speeds from the marker position
        """
        if contourDetection:  # This extra may not be needed now
            if distanceToFrontWall < self.FRONT_BUFFER_STOP:
                LOGGER.info('Too close to wall!')
                self.ROBOTMOVE.stop()
                driveLeft = 0
                driveRight = 0
                speed = 0
                self.reached_marker = True
            elif distanceToFrontWall < self.FRONT_BUFFER_WARN:
                LOGGER.info('Getting closer to wall')
                speed = SpeedSettings.SPEED_SLOW
            else:
                speed = SpeedSettings.SPEED_FAST

            #  Left/right direction - Value between -1.0 to +1.0
            # -1.0 is far left, +1.0 is far right, 0.0 is centre
            direction = (found_x - (self.width / 2)) / (self.width / 2.0)

            if debug_show_steering:
                LOGGER.info(
                    "Distance to front wall: " + str(int(distanceToFrontWall)))
                LOGGER.info(
                    "Found " + ColourBoundaries.
                    COLOUR_NAME_ARRAY[self.colour_array_cntr] + " marker. X: "
                    + str(found_x) + " direction: " + format(direction, '.2f'))

            if direction > 0.0:
                # Turn to robot's right hand side
                LOGGER.info('Forward steer right')
                driveLeft = speed
                driveRight = int(speed * (1.0 -
                                          (direction * self.STEERING_RATE)))
                driveRight = sorted(
                    [self.MIN_SPEED, driveRight, self.MAX_SPEED])[1]
            else:
                # Turn to robot's left hand side
                LOGGER.info('Forward steer left')
                driveLeft = int(speed * (1.0 +
                                         (direction * self.STEERING_RATE)))
                driveLeft = sorted([self.MIN_SPEED, driveLeft,
                                    self.MAX_SPEED])[1]
                driveRight = speed

            # If we need faster turning we may have to change to
            # spin_left or spin_right
            self.ROBOTMOVE.turn_forward(driveLeft, driveRight)

            if debug_show_steering:
                LOGGER.info('Left speed: ' + str(driveLeft) +
                            ' Right speed: ' + str(driveRight))

        else:
            LOGGER.info('No marker')

    def set_speed_to_centre(self, contour_detection, found_x,
                            distanceToFrontWall):
        """
        Calulates the speed of the motors to centre.
        """
        if contour_detection:  # This extra may not be needed now
            if distanceToFrontWall > self.CENTRE_BUFFER_STOP:
                LOGGER.info('Stopping at centre!')
                self.ROBOTMOVE.stop()
                driveLeft = 0
                driveRight = 0
                speed = 0
                self.reached_marker = False

                # Move to next colour
                self.colour_array_cntr = (self.colour_array_cntr + 1) % 4
                LOGGER.info("The colour selector is now " + ColourBoundaries.
                            COLOUR_NAME_ARRAY[self.colour_array_cntr])
            elif distanceToFrontWall > self.CENTRE_BUFFER_WARN:
                LOGGER.info('Getting closer to centre')
                speed = SpeedSettings.SPEED_SLOW
            else:
                speed = SpeedSettings.SPEED_FAST

            #  Left/right direction - Value between -1.0 to +1.0
            # -1.0 is far left, +1.0 is far right, 0.0 is centre
            direction = (found_x - (self.width / 2)) / (self.width / 2.0)

            if debug_show_steering:
                LOGGER.info(
                    'Distance: ' + str(int(distanceToFrontWall)) + ' Found X: '
                    + str(found_x) + ' Direction: ' + str(direction))

            if direction > 0.0:
                # Turn to robot's rear left
                LOGGER.info('Steer reverse left')
                driveLeft = int(speed * (1.0 -
                                         (direction * self.STEERING_RATE)))
                driveLeft = sorted([self.MIN_SPEED, driveLeft,
                                    self.MAX_SPEED])[1]
                driveRight = speed
            else:
                # Turn to robot's right right
                LOGGER.info('Steer reverse right')
                driveLeft = speed
                driveRight = int(speed * (1.0 +
                                          (direction * self.STEERING_RATE)))
                driveRight = sorted(
                    [self.MIN_SPEED, driveRight, self.MAX_SPEED])[1]

            # If we need faster turning we may have to change to
            # spin_left or spin_right
            self.ROBOTMOVE.turn_reverse(driveLeft, driveRight)

            if debug_show_steering:
                LOGGER.info('Left speed: ' + str(driveLeft) +
                            ' Right speed: ' + str(driveRight))

        else:
            LOGGER.info('No marker')

    def spin_to_find_colour(self):
        """
        Spin right until marker found.
        """
        self.ROBOTMOVE.spin_right(SpeedSettings.SPEED_FAST)
        time.sleep(self.SPIN_TIME)
        self.ROBOTMOVE.stop()


def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    """
    LOGGER.info("'Somewhere Over the Rainbow' Starting.")
    LOGGER.info("CTRL^C to terminate program")
    LOGGER.info("Press 'c' to change colour")

    try:
        # Create the object that will process the images
        # passed in to the image_process_entry function
        image_processor = Processor()

        # Start stream process to handle images and
        # pass then to the callback function
        stream_processor = CameraThread.StreamProcessor(
            320, 240, image_processor.image_process_entry, True)

        # Loop indefinitely until we are no longer running
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Somewhere Over the Rainbow'.")

    finally:
        stream_processor.exit_now()
        stream_processor.join()
        image_processor.cleanup()
        cv2.destroyAllWindows()

    LOGGER.info("'Somewhere Over the Rainbow' Finished.")


if __name__ == "__main__":
    main()
