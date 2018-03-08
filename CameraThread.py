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
import cv2
import numpy as np
import picamera
import picamera.array
import RPi.GPIO as GPIO
import SetupConsoleLogger
import ColourBoundaries


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
class StreamProcessor(threading.Thread):

    CAMERA_WIDTH = 320  # 320 x 240 used in PiBorg has been tested with 640 * 480
    CAMERA_HEIGHT = 240
    IMAGE_CENTRE_X = CAMERA_WIDTH / 2.0
    IMAGE_CENTRE_Y = CAMERA_HEIGHT / 2.0
    # Aperture size for median filter based on PiBorg numbers of 5 with 320 * 240
    MED_FILTER_APRTRE_SIZE = 5  # Must be odd number
    # Initial number for down selecting large contours
    # If number too small then will loose circular marker
    NUM_OF_LARGEST_AREA_CONTOURS = 3
    MIN_MARKER_AREA = 100  # Pixels - the final value to be decided from testing

    
    def __init__(self):
        """
        Initialise the parameters required for StreamProcessor thread
        """
        super(StreamProcessor, self).__init__()
        self.event = threading.Event()
        self.exit_now = False
        self.max_processing_delay = 0  # Initialsed for delay calculations
        self.min_processing_delay = 100  # Initialsed for delay calculations
        self.reached_marker = False
   
        self.camera = picamera.PiCamera()
        # Camera resolution defaults to the monitors resolution,
        # but needs to be lower for speed of processing
        self.camera.resolution = (self.CAMERA_WIDTH, self.CAMERA_HEIGHT)
        self.camera.framerate = 10  # If not set then defaults to 30fps
        self.camera.vflip = True  # Needed for mounting of camera on pan/tilt
        self.camera.hflip = True  # Needed for mounting of camera on pan/tilt
        LOGGER.info('Waiting for the camera to wake up ...')
        # Allow the camera time to warm-up
        time.sleep(2)  # This is the value/line used in the PiBorg example
        self.stream = picamera.array.PiRGBArray(self.camera)

        self.start() # starts the thread by calling the run method.

    def __del__(self):
        """
        Destructor
        """
        LOGGER.info("Image processing delays min and max " +
            format(self.min_processing_delay, '.2f') + " " +
            format(self.max_processing_delay, '.2f') + " sec")
        cv2.destroyAllWindows()
        del self.camera  # Is this needed?
        
    def run(self):
        """
        The run() method is the entry point for a thread
        This method runs in a separate thread
        """
        LOGGER.info("Starting the Stream Processing thread")

        while not self.exit_now:
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
        
        LOGGER.info("Finshed the Stream Processing thread")

    def process_image(self, image):
        """
        The main image processing function
        """
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


# Image capture thread
class ImageCapture(threading.Thread):

    def __init__(self, stream_processor):
        """
        Initialise the parameters required for the Image Capture thread
        """
        super(ImageCapture, self).__init__()
        self._stream_processor = stream_processor
        self._exit_now = False
        self.start() # starts the thread by calling the run method.

    def run(self):
        """
        The run() method is the entry point for a thread
        This method runs in a separate thread
        """
        LOGGER.info("Starting the ImageCapture thread")
        
        self._stream_processor.camera.capture_sequence(
            self.trigger_stream(), format='bgr', use_video_port=True)
            
        self._stream_processor.exit_now = True
        self._stream_processor.join()  # The join() waits for threads to terminate

        LOGGER.info("Finshed the ImageCapture thread")

    def exit_now(self):
        """
        Request the thread to exit
        """
        self._exit_now = True

    # Stream delegation loop
    def trigger_stream(self):

        while not self._exit_now:
            if self._stream_processor.event.is_set():
                time.sleep(0.01)
            else:
                yield self._stream_processor.stream
                self._stream_processor.event.set()


def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    """
    LOGGER.info("'Somewhere Over the Rainbow' Starting.")
    LOGGER.info("CTRL^C to terminate program")
    LOGGER.info("Press 'c' to change colour")
                
    try:
        # Start stream process to handle images
        stream_processor = StreamProcessor()

        # Start the thread to generate images
        capture_thread = ImageCapture(stream_processor)

        # Loop indefinitely until we are no longer running
        while True:
            # Wait for the interval period
            time.sleep(1)

    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Somewhere Over the Rainbow'.")
    finally:
        capture_thread.exit_now()
        capture_thread.join()
        stream_processor.terminated = True
        stream_processor.join()
    
    LOGGER.info("'Somewhere Over the Rainbow' Finished.")

if __name__ == "__main__":
    main()
