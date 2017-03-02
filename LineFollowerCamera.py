#!/usr/bin/python

"""
Provides ability to use the Raspberry Pi Camera to detect the line
to follow for the line follwowing challenge in PiWars 2017
"""

import logging
import picamera
import RPi.GPIO as GPIO
import ServoController
import SetupConsoleLogger

# Create a logger to both file and stdout
MODULE_LOGGER = logging.getLogger("__main__.LineFollowerCamera")


class LineFollowerCamera(object):

    """
    Provides ability to detect the angle of the line in front of the
    robot.
    """

    def __init__(self):
        """
        Initialises the class
        """
        MODULE_LOGGER.info("Setting up LineFollowerCamera Module")

        # Set camera to point in correct direction
        SERVO_CONTROLLER = ServoController.ServoController()
        SERVO_CONTROLLER.start_servos()
        SERVO_CONTROLLER.set_pan_servo(0)
        SERVO_CONTROLLER.set_tilt_servo(-45)

        # Initise PiCamera
        camera = picamera.PiCamera()

    def getLineAngle(self):
        """
        Get the angle of the line
        """
        Angle = 45.0
        
        return Angle

    def cleanUp(self):
        """
        Stops the servos by calling the function
        """
        SERVO_CONTROLLER.stop_servos()


if __name__ == "__main__":
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)

        AngleCamera = LineFollowerCamera()

        CurrentAngle = AngleCamera.getLineAngle()

        MODULE_LOGGER.info("Current angle of the line: " +
                           str(int(CurrentAngle)) + " deg")
                           
    except KeyboardInterrupt:
        pass
    finally:
        MODULE_LOGGER.info("End of test")
