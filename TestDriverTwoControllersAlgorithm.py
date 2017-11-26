#!/usr/bin/python

"""
Provides an mechanism to drive the vehicle using two controllers.
"""

import logging
import time
import MotorController
import SetupConsoleLogger
import GPIOLayout
import KeyboardCharacterReader
import cwiid


# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
FRONTDRIVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_PIN)

# Initialise motors
REARDRIVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_PIN)


def one_wheel():
    """
    Performs single wheel movements
    """


def main():
    """
    Performs the main driving algo test
    """
    FRONTDRIVE.stop()
    REARDRIVE.stop()
    FRONTDRIVE.stop()
    REARDRIVE.stop()
  
    speed_list = [MotorController.SPEED_FASTEST,
                  MotorController.SPEED_MEDIUM,
                  MotorController.SPEED_VERYVERYSLOW]

    for x in speed_list:
		LOGGER.info("Speed " + str(x))
		FRONTDRIVE.stop()
		REARDRIVE.stop()

		LOGGER.info("forward")
		FRONTDRIVE.forward(x)
		REARDRIVE.forward(x)
		time.sleep(5)

		LOGGER.info("reverse")
		FRONTDRIVE.reverse(x)
		REARDRIVE.reverse(x)
		time.sleep(3)

		LOGGER.info("spin_right")
		FRONTDRIVE.spin_right(x)
		REARDRIVE.spin_right(x)
		time.sleep(5)

		LOGGER.info("spin_left")
		FRONTDRIVE.spin_left(x)
		REARDRIVE.spin_left(x)
		time.sleep(5)

		LOGGER.info("turn_forward r")
		FRONTDRIVE.turn_forward(x, 0)
		REARDRIVE.turn_forward(x, 0)
		time.sleep(5)

		LOGGER.info("turn_forward l")
		FRONTDRIVE.turn_forward(0, x)
		REARDRIVE.turn_forward(0, x)
		time.sleep(5)

		LOGGER.info("turn_reverse r")
		FRONTDRIVE.turn_reverse(x, 0)
		REARDRIVE.turn_reverse(x, 0)
		time.sleep(5)

		LOGGER.info("turn_reverse l")
		FRONTDRIVE.turn_reverse(0, x)
		REARDRIVE.turn_reverse(0, x)
		time.sleep(5)

    
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the drive test")
    finally:
        LOGGER.info("Drive Test Finished")
        FRONTDRIVE.stop()
        REARDRIVE.stop()
        FRONTDRIVE.cleanup()
