#!/usr/bin/python

"""
MinimalMazeAlgorithm.py
This code is for use in the Minimal Maze - PiWars 2017 challenge
http://piwars.org/
"""

import logging
import time
import DualMotorController
import UltrasonicSensor
import GPIOLayout
import KeyboardCharacterReader

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
# SetupConsoleLogger.setup_console_logger(LOGGER)

CORRECTION_TIME = 0.025  # Angle correction delay time in seconds

# Initialise motors
ROBOTMOVE = DualMotorController.DualMotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)


def turn_right(ultrasonic_sensor_side):

    while True:
        ROBOTMOVE.stop()
        # time.sleep(CORRECTION_TIME)

        distance_side = ultrasonic_sensor_side.measurement()

        # Track distances
        LOGGER.info("Distance (side): %.2f cm", distance_side)
        print(distance_side)

        ROBOTMOVE.spin_right(MotorController.SPEED_MEDIUM)
        time.sleep(CORRECTION_TIME)


def main():
    """
    Performs the main line following algorithm
    """
    LOGGER.info("Turn Algo test press 'Space' key.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_GPIO,
        GPIOLayout.SONAR_LEFT_TX_GPIO)

    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    turn_right(view_left)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the race")
    finally:
        LOGGER.info("Turn Algo test Finished")
        ROBOTMOVE.cleanup()
