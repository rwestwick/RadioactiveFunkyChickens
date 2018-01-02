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
    def ProcessImage(self, image, colour):

        # Blur the image
        image = cv2.medianBlur(image, 5)

        # Convert the image from 'BGR' to HSV colour space
        image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)


# Initialise objects and constants

# Create a logger to both file and stdout
LOGGER = logging.getLogger("__name__")
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise servos
SERVO_CONTROLLER = ServoController.ServoController()


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
