#!/usr/bin/python
"""
JustDrive.py needs to be run from the command line
This code is for the remote-controlled challenges
http://piwars.org/
"""

# Import required libraries
import time
import logging
import SetupConsoleLogger
import WiimoteNunchukControllerThread

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER, logging.DEBUG)


def main():
    """
    """
    LOGGER.info("Just Drive")
    WIIMOTE_CONTROLLER = \
        WiimoteNunchukControllerThread.WiimoteNunchukControllerThread(
        None, None, None, None)
    try:
        WIIMOTE_CONTROLLER.start()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        LOGGER.info("Stopping the Wiimote Controller thread")
    finally:
        WIIMOTE_CONTROLLER.exit_now()
        WIIMOTE_CONTROLLER.join()
        LOGGER.info("'Just Drive' Finished.")


if __name__ == "__main__":
    main()
