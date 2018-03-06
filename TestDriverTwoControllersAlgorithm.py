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
import SpeedSettings

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
DMCONTROLLER = DualMotorController.DualMotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)


def keyboard_movements():
    """
    """
    speed_list = [
        SpeedSettings.SPEED_FASTEST, SpeedSettings.SPEED_FAST,
        SpeedSettings.SPEED_MEDIUM, SpeedSettings.SPEED_SLOW,
        SpeedSettings.SPEED_VERYSLOW, SpeedSettings.SPEED_VERYVERYSLOW
    ]

    pos = 0
    x = speed_list[pos]
    LOGGER.info("Speed " + str(x))
    DMCONTROLLER.stop()

    # Start the challenge
    while True:
        keyp = KeyboardCharacterReader.readkey()

        if keyp == 'q':
            LOGGER.info("Front Left Forwards")
            DMCONTROLLER.front_left_forward(x)
        elif keyp == 'a':
            LOGGER.info("Front Left Stop")
            DMCONTROLLER.front_left_forward(0)
        elif keyp == 'z':
            LOGGER.info("Front Left Backwards")
            DMCONTROLLER.front_left_backward(x)

        if keyp == 'w':
            LOGGER.info("Front Right Forwards")
            DMCONTROLLER.front_right_forward(x)
        elif keyp == 's':
            LOGGER.info("Front Right Stop")
            DMCONTROLLER.front_right_forward(0)
        elif keyp == 'x':
            LOGGER.info("Front Right Backwards")
            DMCONTROLLER.front_right_backward(x)

        if keyp == 'e':
            LOGGER.info("Back Left Forwards")
            DMCONTROLLER.rear_left_forward(x)
        elif keyp == 'd':
            LOGGER.info("Back Left Stop")
            DMCONTROLLER.rear_left_forward(0)
        elif keyp == 'c':
            LOGGER.info("Back Left Backwards")
            DMCONTROLLER.rear_left_backward(x)

        if keyp == 'r':
            LOGGER.info("Back Right Forwards")
            DMCONTROLLER.rear_right_forward(x)
        elif keyp == 'f':
            LOGGER.info("Back Right Stop")
            DMCONTROLLER.rear_right_forward(0)
        elif keyp == 'v':
            LOGGER.info("Back Right Backwards")
            DMCONTROLLER.rear_right_backward(x)

        if ord(keyp) == 16:
            LOGGER.info("forward")
            DMCONTROLLER.forward(x)
        elif ord(keyp) == 17:
            LOGGER.info("reverse")
            DMCONTROLLER.reverse(x)
        elif ord(keyp) == 19:
            LOGGER.info("left")
            DMCONTROLLER.spin_left(x)
        elif ord(keyp) == 18:
            LOGGER.info("right")
            DMCONTROLLER.spin_right(x)
        elif keyp == ' ':
            LOGGER.info("stop")
            DMCONTROLLER.stop()

        elif keyp == '+':
            if pos > 0:
                pos -= 1
                x = speed_list[pos]
            LOGGER.info("Speed " + str(x))

        elif keyp == '-':
            if pos < len(speed_list) - 1:
                pos += 1
                x = speed_list[pos]
            LOGGER.info("Speed " + str(x))

        # To end if Ctrl-C pressed
        elif ord(keyp) == 3:
            LOGGER.info("Break control loop!")
            break


def one_wheel_movements():
    """
    Performs single wheel movements
    """
    x = SpeedSettings.SPEED_FASTEST
    LOGGER.info("Speed " + str(x))
    DMCONTROLLER.stop()
    time.sleep(5)

    LOGGER.info("front_left_forward")
    DMCONTROLLER.front_left_forward(x)
    time.sleep(5)
    DMCONTROLLER.stop()

    LOGGER.info("front_left_backward")
    DMCONTROLLER.front_left_backward(x)
    time.sleep(5)
    DMCONTROLLER.stop()

    LOGGER.info("front_right_forward")
    DMCONTROLLER.front_right_forward(x)
    time.sleep(5)
    DMCONTROLLER.stop()

    LOGGER.info("front_right_backward")
    DMCONTROLLER.front_right_backward(x)
    time.sleep(5)
    DMCONTROLLER.stop()

    LOGGER.info("rear_left_forward")
    DMCONTROLLER.rear_left_forward(x)
    time.sleep(5)
    DMCONTROLLER.stop()

    LOGGER.info("rear_left_backward")
    DMCONTROLLER.rear_left_backward(x)
    time.sleep(5)
    DMCONTROLLER.stop()

    LOGGER.info("rear_right_forward")
    DMCONTROLLER.rear_right_forward(x)
    time.sleep(5)
    DMCONTROLLER.stop()

    LOGGER.info("rear_right_backward")
    DMCONTROLLER.rear_right_backward(x)
    time.sleep(5)


def main_movements():
    """
    Performs the main driving algo test
    """
    DMCONTROLLER.stop()

    speed_list = [
        SpeedSettings.SPEED_FASTEST, SpeedSettings.SPEED_MEDIUM,
        SpeedSettings.SPEED_VERYVERYSLOW
    ]

    for x in speed_list:
        LOGGER.info("Speed " + str(x))
        DMCONTROLLER.stop()

        LOGGER.info("forward")
        DMCONTROLLER.forward(x)
        time.sleep(3)

        LOGGER.info("reverse")
        DMCONTROLLER.reverse(x)
        time.sleep(3)

        LOGGER.info("spin_right")
        DMCONTROLLER.spin_right(x)
        time.sleep(3)

        LOGGER.info("spin_left")
        DMCONTROLLER.spin_left(x)
        time.sleep(3)


if __name__ == "__main__":
    try:
        keyboard_movements()
        # main_movements()
        # one_wheel_movements()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the drive test")
    finally:
        LOGGER.info("Drive Test Finished")
        DMCONTROLLER.stop()
        time.sleep(1)
        DMCONTROLLER.cleanup()
