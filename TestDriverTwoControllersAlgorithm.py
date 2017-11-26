#!/usr/bin/python

"""
Provides an mechanism to drive the vehicle using two controllers.
"""

import logging
import time
import DualMotorController
import SetupConsoleLogger
import GPIOLayout
import KeyboardCharacterReader
import cwiid


# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
DMCONTROLLER = DualMotorController.DualMotorController(
                GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_PIN,
                GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_PIN,
                GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_PIN,
                GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_PIN,
                GPIOLayout.MOTOR_LEFT_REAR_FORWARD_PIN,
                GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_PIN,
                GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_PIN,
                GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_PIN)


def one_wheel():
    """
    Performs single wheel movements
    """


def main_movements():
    """
    Performs the main driving algo test
    """
    DMCONTROLLER.stop()
  
    speed_list = [DualMotorController.SPEED_FASTEST,
                  DualMotorController.SPEED_MEDIUM,
                  DualMotorController.SPEED_VERYVERYSLOW]

    for x in speed_list:
		LOGGER.info("Speed " + str(x))
		DMCONTROLLER.stop()

		LOGGER.info("forward")
		DMCONTROLLER.forward(x)
		time.sleep(5)

		LOGGER.info("reverse")
		DMCONTROLLER.reverse(x)
		time.sleep(3)

		LOGGER.info("spin_right")
		DMCONTROLLER.spin_right(x)
		time.sleep(5)

		LOGGER.info("spin_left")
		DMCONTROLLER.spin_left(x)
		time.sleep(5)


    
if __name__ == "__main__":
    try:
        main_movements()
        one_wheel()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the drive test")
    finally:
        LOGGER.info("Drive Test Finished")
        DMCONTROLLER.stop()
        DMCONTROLLER.cleanup()
