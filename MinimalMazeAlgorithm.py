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
FRONT_BUFFER = 20 # Shortest distance to front (cm)
SIDE_BUFFER = 10 # Shortest distance to side (cm)
CORRECTION_TIME = 0.2 # Angle correction delay time in seconds
FIRST_WALL_LENGTH = 122 # Length of first wall (cm)
SECOND_WALL_LENGTH = 204 # Length of first wall (cm)
MIDDLE_GAP_WIDTH = 36 # Gap between each pair of walls at the middle "S" point
EXIT_GAP = 40 # Gap between each pair of walls at exit

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)


def turn_left():
    ROBOTMOVE.spin_left(MotorController.SPEED_MEDIUM, 0)
    time.sleep(1)
    ROBOTMOVE.stop()


def turn_right():
    ROBOTMOVE.spin_right(MotorController.SPEED_MEDIUM, 0)
    time.sleep(1)
    ROBOTMOVE.stop()


def follow_wall(ultrasonic_sensor_side, ultrasonic_sensor_front):
    # Drive forward
    ROBOTMOVE.forward(MotorController.SPEED_MEDIUM)

    LOGGER.info("Following Wall")

    # Track first right wall round bend
    while True:
        # Take measurements
        distance_side = ultrasonic_sensor_side.measurement()
        distance_stop = ultrasonic_sensor_front.measurement()

        # Track distances
        LOGGER.info("Distance: " + str(int(distance_side)) + " cm")

        # Decide which way to steer
        if distance_side > SIDE_BUFFER:
            LOGGER.info("Steering right")
            ROBOTMOVE.turn_forward(MotorController.SPEED_MEDIUM, 0)
            time.sleep(CORRECTION_TIME)
            ROBOTMOVE.forward(MotorController.SPEED_MEDIUM)
            time.sleep(CORRECTION_TIME)
        elif distance_side < SIDE_BUFFER:
            LOGGER.info("Steering left")
            ROBOTMOVE.turn_forward(0, MotorController.SPEED_MEDIUM)
            time.sleep(CORRECTION_TIME)
            ROBOTMOVE.forward(MotorController.SPEED_MEDIUM)
            time.sleep(CORRECTION_TIME)

        # Will robot follow round the curve? Could use colour of walls and camera
        # If yes, then stop when too close to bottom wall
        if distance_stop < FRONT_BUFFER:
            break


def main():
    """
    Performs the main line following algorithm
    """
    LOGGER.info("Minimal Maze")

    # Waiting for start of race
    LOGGER.info("To start maze solving following press 'Space' key.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_PIN,
        GPIOLayout.SONAR_LEFT_TX_PIN)
    view_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN,
        GPIOLayout.SONAR_RIGHT_TX_PIN)
    view_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_PIN)

    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    follow_wall(view_left, view_front)
    turn_right()

    follow_wall(view_left, view_front)
    turn_right()

    follow_wall(view_left, view_front)
    turn_right()

    follow_wall(view_left, view_right)
    follow_wall(view_right, view_front)
    turn_left()

    follow_wall(view_right, view_front)
    turn_left()

    follow_wall(view_right, view_front)
    turn_left()

    follow_wall(view_right, view_front)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the race")
    finally:
        LOGGER.info("Maze Solver Finished")
        ROBOTMOVE.cleanup()
