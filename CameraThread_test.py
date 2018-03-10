#!/usr/bin/python
"""
Provides the test functionality for the Camera Thread
"""

import logging
import time
import cv2
import SetupConsoleLogger
import CameraThread

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER, logging.DEBUG)


# Image capture thread
class ImageProcessor(object):
    """
    """

    def __init__(self):
        """
        Constructor
        """

    def image_process_entry(self, bgr_image):
        """
        Called each time an image can be processed
        """
        # Blur the image
        MED_FILTER_APRTRE_SIZE = 5  # Must be odd number
        bgr_blur_image = cv2.medianBlur(bgr_image, MED_FILTER_APRTRE_SIZE)

        # Convert the image from 'BGR' to HSV colour space
        hsv_image = cv2.cvtColor(bgr_blur_image, cv2.COLOR_BGR2HSV)

        cv2.imshow('Blured HSV Image', hsv_image)

        # Capture a key press. The function waits argument in ms
        # for any keyboard event
        # For some reason image does not show without this!
        cv2.waitKey(1) & 0xFF


def main():
    """
    Performs the "Camera Capture and stream mechanism" test
    """
    MODULE_LOGGER.info("'Camera Capture and stream mechanism' Starting.")
    MODULE_LOGGER.info("CTRL^C to terminate program")

    try:
        # Create the object that will process the images
        # passed in to the image_process_entry function
        image_processor = ImageProcessor()

        # Start stream process to handle images and
        # pass then to the callback function
        stream_processor = CameraThread.StreamProcessor(
            320, 240, image_processor.image_process_entry, True)

        # Wait for the interval period for finishing
        time.sleep(10)

    except KeyboardInterrupt:
        MODULE_LOGGER.info("Stopping 'Camera Capture and stream mechanism'.")

    finally:
        stream_processor.exit_now()

    MODULE_LOGGER.info("'Camera Capture and stream mechanism' Finished.")


if __name__ == "__main__":
    main()
