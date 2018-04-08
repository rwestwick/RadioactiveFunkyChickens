#!/usr/bin/python
"""
Provides the test functionality for the Camera Thread
"""

import logging
import time
import cv2
import SetupConsoleLogger
import CameraThread
from ar_markers.hamming.detect import detect_markers

LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER, logging.DEBUG)


class Processor(object):
    """
    """

    def __init__(self):
        """
        Constructor
        """
        LOGGER.debug("Processor constructor called")

        self.width = 640
        self.height = 480

    def __del__(self):
        """
        Destructor
        """
        self.cleanup()

    def cleanup(self):
        """
        Cleanup
        """

    def image_process_entry(self, image, width, height):
        """
        Called each time an image can be processed
        """
        self.width = width
        self.height = height

        markers = detect_markers(image)
        for marker in markers:
            marker.highlite_marker(image)
            LOGGER.debug("Found a marker " + str(marker.id))

        cv2.imshow('Marker Hightlight', image)
        cv2.waitKey(1) & 0xFF


def main():
    """
    Performs the "Camera Capture and stream mechanism" test
    """
    LOGGER.info("'Camera Capture and stream mechanism' Starting.")
    LOGGER.info("CTRL^C to terminate program")

    try:
        # Create the object that will process the images
        # passed in to the image_process_entry function
        image_processor = Processor()

        # Start stream process to handle images and
        # pass then to the callback function
        stream_processor = CameraThread.StreamProcessor(
            640, 480, image_processor.image_process_entry, False)

        # Wait for the interval period for finishing
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Camera Capture and stream mechanism'.")

    finally:
        stream_processor.exit_now()
        stream_processor.join()
        image_processor.cleanup()
        cv2.destroyAllWindows()

    LOGGER.info("'Camera Capture and stream mechanism' Finished.")


if __name__ == "__main__":
    main()
