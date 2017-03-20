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
FRONT_BUFFER_WARN = 30 # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 20 # Shortest distance to front (cm)
SIDE_BUFFER = 12 # Shortest distance to side (cm)
CORRECTION_TIME = 0.1 # Angle correction delay time in seconds


# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)


def turn_left():
    ROBOTMOVE.spin_left(MotorController.SPEED_MEDIUM)
    time.sleep(0.7)
    ROBOTMOVE.stop()


def turn_right():
    ROBOTMOVE.spin_right(MotorController.SPEED_MEDIUM)
    time.sleep(0.7)
    ROBOTMOVE.stop()


def follow_wall(ultrasonic_sensor_side, ultrasonic_sensor_front):
    # Drive forward
    ROBOTMOVE.forward(MotorController.SPEED_SLOW)

    LOGGER.info("====  Following Wall ====")

    # Track first right wall round bend
    while True:
        forward_speed = MotorController.SPEED_MEDIUM
        turn_speed = MotorController.SPEED_FASTEST

        # Take measurements
        distance_side = ultrasonic_sensor_side.measurement()
        distance_stop = ultrasonic_sensor_front.measurement()

        # Track distances
        LOGGER.info("Distance (side): " + str(int(distance_side)) + " cm")
        LOGGER.info("Distance (stop): " + str(int(distance_stop)) + " cm")

        # Will robot follow round the curve? Could use colour of walls
        # and camera
        # If yes, then stop when too close to bottom wall
        if distance_stop < FRONT_BUFFER_STOP:
            LOGGER.info("Stop!")
            break

        if distance_stop < FRONT_BUFFER_WARN:
            LOGGER.info("Slow down!")
            forward_speed = MotorController.SPEED_VERYVERYSLOW
            #turn_speed = MotorController.SPEED_MEDIUM

        # Decide which way to steer
        if distance_side < SIDE_BUFFER:
            LOGGER.info("Steering right")
            ROBOTMOVE.turn_forward(turn_speed, 0)
            time.sleep(CORRECTION_TIME)
            ROBOTMOVE.forward(forward_speed)
            time.sleep(CORRECTION_TIME)
        elif distance_side > SIDE_BUFFER:
            LOGGER.info("Steering left")
            ROBOTMOVE.turn_forward(0, turn_speed)
            time.sleep(CORRECTION_TIME)
            ROBOTMOVE.forward(forward_speed)
            time.sleep(CORRECTION_TIME)


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

    follow_wall(view_left, view_front)
    turn_right()
    turn_right()
    turn_right()
    turn_right()
    ROBOTMOVE.stop()

    wibble

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
