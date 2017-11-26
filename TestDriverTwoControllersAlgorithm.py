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

    
def main():
    """
    Performs the main driving algo test
    """
    FRONTDRIVE.stop()
    REARDRIVE.stop()
    FRONTDRIVE.stop()
    REARDRIVE.stop()
    
    FRONTDRIVE.forward(SPEED_MEDIUM)
    REARDRIVE.forward(SPEED_MEDIUM)
    time.sleep(2)
    
    FRONTDRIVE.spin_right(SPEED_MEDIUM)
    REARDRIVE.spin_right(SPEED_MEDIUM)
    time.sleep(2)

    

    
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
