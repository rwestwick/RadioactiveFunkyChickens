#!/usr/bin/python

"""
MinimalMazeAlgorithm.py
This code is for use in the Minimal Maze - PiWars 2017 challenge
http://piwars.org/
"""

import logging
import time
import MotorController
import UltrasonicSensor
import SetupConsoleLogger
import GPIOLayout
import KeyboardCharacterReader


# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set initial constant values
ROBOT_LENGTH = 20 # Length of the robot (cm)
ROBOT_WIDTH = 14 # Width of the robot (cm)
FRONT_BUFFER = 10 # Shortest distance to front (cm)
SIDE_BUFFER = 10 # Shortest distance to side (cm)
CORRECTION_TIME = 0.2 # Angle correction delay time in seconds
FIRST_WALL_LENGTH = 122 # Length of first wall (cm)
SECOND_WALL_LENGTH = 204 # Length of first wall (cm)

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)


def main():
    """
    Performs the main line following algorithm
    """
    LOGGER.info("Minimal Maze")

    # Waiting for start of race
    LOGGER.info("To start maze solving following press 'Space' key.")

    # Create necessary sensor objects
    viewLeft = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_PIN,
        GPIOLayout.SONAR_LEFT_TX_PIN)
    viewRight = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN,
        GPIOLayout.SONAR_RIGHT_TX_PIN)
    viewFront = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_PIN)

    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    # Take initial measurements
    distanceLeft = viewLeft.measurement()
    distanceRight = viewRight.measurement()
    distanceFront = viewFront.measurement()

    # Drive forward
    ROBOTMOVE.forward(MotorController.SPEED_MEDIUM)

    # Steer to first wall
    while distanceFront > FRONT_BUFFER:
        
	# Take measurements
	distanceLeft = viewLeft.measurement()
	distanceRight = viewRight.measurement()
	distanceFront = viewFront.measurement()
	
        # Track distances
        LOGGER.info("Left: " + str(int(distanceLeft)) + " - Right: " + str(int(distanceRight)) +
                    " - Forward: " +	str(int(distanceFront)) + " cm")

        # time.sleep(1)

        # Decide which way to steer
        if (distanceLeft < SIDE_BUFFER):
            LOGGER.info("Steering right")
            ROBOTMOVE.turn_forward(MotorController.SPEED_MEDIUM, 0)
            time.sleep(CORRECTION_TIME)
            ROBOTMOVE.forward(MotorController.SPEED_MEDIUM)
            time.sleep(CORRECTION_TIME)
        elif (distanceRight < SIDE_BUFFER):
            LOGGER.info("Steering left")
            ROBOTMOVE.turn_forward(0, MotorController.SPEED_MEDIUM)
            time.sleep(CORRECTION_TIME)
            ROBOTMOVE.forward(MotorController.SPEED_MEDIUM)
            time.sleep(CORRECTION_TIME)

    ROBOTMOVE.stop()

    # Spin right till side is looking down entrance
    while (distanceRight < FIRST_WALL_LENGTH) and (distanceFront < (SECOND_WALL_LENGTH - SIDE_BUFFER - ROBOT_LENGTH)):
        LOGGER.info("Spining right")
        ROBOTMOVE.spin_right(MotorController.SPEED_MEDIUM)

    ROBOTMOVE.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the race")
    finally:
        LOGGER.info("Maze Solver Finished")
        ROBOTMOVE.cleanup()
