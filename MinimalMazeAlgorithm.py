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
import SetupConsoleLogger
import GPIOLayout
import KeyboardCharacterReader
import SpeedSettings

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set initial constant values
FRONT_BUFFER_WARN = 35  # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 25  # Shortest distance to front (cm)
SIDE_BUFFER = 10  # Shortest distance to side (cm)
CORRECTION_TIME = 0.15  # Angle correction delay time in seconds
FORWARD_TIME = 0.1  # Angle correction delay time in seconds
TURN_DELAY = 0.65

# Initialise motors
ROBOTMOVE = DualMotorController.DualMotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_PIN,
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_PIN)


def turn_left(delay):
    LOGGER.info("Left Turn")
    ROBOTMOVE.spin_left(SpeedSettings.SPEED_MEDIUM)
    time.sleep(delay)
    ROBOTMOVE.stop()


def turn_right(delay):
    LOGGER.info("Right Turn")
    ROBOTMOVE.spin_right(SpeedSettings.SPEED_MEDIUM)
    time.sleep(delay)
    ROBOTMOVE.stop()


def follow_wall(side_buffer, ultrasonic_sensor_left, ultrasonic_sensor_right,
                ultrasonic_sensor_front):

    LOGGER.info("====  Following Wall ====")
    LOGGER.info("Distance (side warning): " + str(side_buffer) + " cm")

    # Track first right wall round bend
    while True:
        forward_speed = SpeedSettings.SPEED_SLOW
        turn_speed = SpeedSettings.SPEED_FASTEST

        # Take measurements
        if ultrasonic_sensor_left is not None:
            distance_left = ultrasonic_sensor_left.measurement()
            # LOGGER.info("Distance (left side): " +
            # format(distance_left, '.2f') + " cm")

        if ultrasonic_sensor_right is not None:
            distance_right = ultrasonic_sensor_right.measurement()
            # LOGGER.info("Distance (right side): " +
            # format(distance_right, '.2f') + " cm")

        distance_stop = ultrasonic_sensor_front.measurement()
        # LOGGER.info("Distance (stop): " +
        # format(distance_stop, '.2f') + " cm")

        # Will robot follow round the curve? Could use colour of walls
        # and camera
        # If yes, then stop when too close to bottom wall
        if distance_stop < FRONT_BUFFER_STOP:
            LOGGER.info("Stop!")
            break

        if distance_stop < FRONT_BUFFER_WARN:
            LOGGER.info("Slowing down")
            forward_speed = SpeedSettings.SPEED_VERYVERYSLOW

        if ultrasonic_sensor_left is not None:
            # Decide which way to steer
            if distance_left < (side_buffer - 1):
                LOGGER.info("LS Steering right")
                ROBOTMOVE.turn_forward(turn_speed, 0)
                time.sleep(CORRECTION_TIME)
                ROBOTMOVE.forward(forward_speed)

            elif distance_left > (side_buffer + 1):
                LOGGER.info("LS Steering left")
                ROBOTMOVE.turn_forward(0, turn_speed)
                time.sleep(CORRECTION_TIME)
                ROBOTMOVE.forward(forward_speed)

            else:
                LOGGER.info("LS Forward")
                ROBOTMOVE.forward(forward_speed)
                time.sleep(FORWARD_TIME)

        elif ultrasonic_sensor_right is not None:
            # Decide which way to steer
            if distance_right < (side_buffer - 1):
                LOGGER.info("RS Steering left")
                ROBOTMOVE.turn_forward(0, turn_speed)
                time.sleep(CORRECTION_TIME)
                ROBOTMOVE.forward(forward_speed)

            elif distance_right > (side_buffer + 1):
                LOGGER.info("RS Steering right")
                ROBOTMOVE.turn_forward(turn_speed, 0)
                time.sleep(CORRECTION_TIME)
                ROBOTMOVE.forward(forward_speed)

            else:
                LOGGER.info("RS Forward")
                ROBOTMOVE.forward(forward_speed)
                time.sleep(FORWARD_TIME)
        else:
            LOGGER.info("Forward")
            ROBOTMOVE.forward(forward_speed)
            time.sleep(FORWARD_TIME)

    LOGGER.info("Wall follower ended")


def main():
    """
    Performs the main line following algorithm
    """
    LOGGER.info("Minimal Maze")

    # Waiting for start of race
    LOGGER.info("To start maze solving following press 'Space' key.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(GPIOLayout.SONAR_LEFT_RX_PIN,
                                                  GPIOLayout.SONAR_LEFT_TX_PIN)
    view_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN, GPIOLayout.SONAR_RIGHT_TX_PIN)
    view_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_PIN)

    # SIDE_BUFFER = view_left.measurement()
    LEFT_SIDE_BUFFER = 9.0
    RIGHT_SIDE_BUFFER = 10.0

    LOGGER.info("Distance view_left at start " +
                format(view_left.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_right at start " +
                format(view_right.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_front at start " +
                format(view_front.measurement(), '.2f') + " cm")

    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    LOGGER.info("First line")
    follow_wall(LEFT_SIDE_BUFFER, view_left, None, view_front)
    turn_right(TURN_DELAY)

    LOGGER.info("")
    LOGGER.info("Second line")

    follow_wall(LEFT_SIDE_BUFFER, view_left, None, view_front)
    turn_right(TURN_DELAY)

    LOGGER.info("")
    LOGGER.info("Third towards 45 deg")

    follow_wall(SIDE_BUFFER, view_left, None, view_front)
    turn_right(TURN_DELAY)

    LOGGER.info("")
    LOGGER.info("Fourth towards last 45 deg")

    follow_wall(LEFT_SIDE_BUFFER, None, None, view_front)
    turn_left(TURN_DELAY)

    LOGGER.info("")
    LOGGER.info("Fifth short ")

    follow_wall(RIGHT_SIDE_BUFFER, None, view_right, view_front)
    turn_left(TURN_DELAY)

    LOGGER.info("")
    LOGGER.info("Sixth short")

    follow_wall(RIGHT_SIDE_BUFFER, None, view_right, view_front)
    turn_left(TURN_DELAY)

    LOGGER.info("")
    LOGGER.info("Seventh long")

    follow_wall(RIGHT_SIDE_BUFFER, None, view_right, view_front)
    ROBOTMOVE.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the race")
    finally:
        LOGGER.info("Maze Solver Finished")
        ROBOTMOVE.cleanup()
