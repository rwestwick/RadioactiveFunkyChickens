#!/usr/bin/python
"""
JustDrive.py needs to be run from the command line
This code is for the remote-controlled The Duck Shoot PiWars 2018 challenge
http://piwars.org/
"""

# Import required libraries
import time
import logging
import math
import cwiid
import SetupConsoleLogger
import GPIOLayout
import WiimoteNunchukControllerThread


# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER, logging.DEBUG)


def main():
    """
    """
    LOGGER.info("Just Drive")
    WIIMOTE_CONTROLLER = WiimoteNunchukControllerThread.WiimoteNunchukControllerThread(
            None,
            None,
            None,
            None)
    try:
        LOGGER.info("starting")
        WIIMOTE_CONTROLLER.start()
        time.sleep(600)

    except KeyboardInterrupt:
        LOGGER.info("Stopping the Wiimote Controller thread")
    finally:
        WIIMOTE_CONTROLLER.exit_now()
        WIIMOTE_CONTROLLER.join()
        LOGGER.info("'Just Drive' Finished.")


if __name__ == "__main__":
    main()
