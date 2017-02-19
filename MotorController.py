#!/usr/bin/python

"""
Provides ability to control the motors on the robot.
"""

import RPi.GPIO as GPIO
import time
import logging
import SetupConsoleLogger

MODULE_LOGGER = logging.getLogger("__main__.MotorController")


class MotorController(object):

    """
    Provides ability to control the motors on the robot.
    """

    # Define some basic speed settings
    SPEED_FASTEST = 100
    SPEED_FAST = 80
    SPEED_MEDIUM = 50
    SPEED_SLOW = 40
    SPEED_VERYSLOW = 30
    SPEED_VERYVERYSLOW = 20

    # ======================================================================
    # Existing robohat.py sensor pins - J5 Pin Numbers
    # ======================================================================
    # Pins 35, 36 Left Motor
    # Pins 32, 33 Right Motor
    LEFT_FORWARD = 36
    LEFT_BACKWARD = 35
    RIGHT_FORWARD = 33
    RIGHT_BACKWARD = 32
    START_FREQ = 25

    def __init__(self):
        """
        Initialises GPIO pins
        """
        MODULE_LOGGER.info("Setting up MotorController Module")

        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)

        # use pwm on inputs so motors don't go too fast
        GPIO.setup(self.LEFT_FORWARD, GPIO.OUT)
        self.motor_left_forward = GPIO.PWM(self.LEFT_FORWARD,
                                           self.START_FREQ)
        self.motor_left_forward.start(0)

        GPIO.setup(self.LEFT_BACKWARD, GPIO.OUT)
        self.motor_left_backward = GPIO.PWM(self.LEFT_BACKWARD,
                                            self.START_FREQ)
        self.motor_left_backward.start(0)

        GPIO.setup(self.RIGHT_FORWARD, GPIO.OUT)
        self.motor_right_forward = GPIO.PWM(self.RIGHT_FORWARD,
                                            self.START_FREQ)
        self.motor_right_forward.start(0)

        GPIO.setup(self.RIGHT_BACKWARD, GPIO.OUT)
        self.motor_right_backward = GPIO.PWM(self.RIGHT_BACKWARD,
                                             self.START_FREQ)
        self.motor_right_backward.start(0)

    def cleanup(self):
        """
        Sets all motors off and sets GPIO to standard values
        """
        self.stop()
        time.sleep(1)
        GPIO.cleanup()

    def stop(self):
        """
        Causes the Robot to stop all motors
        """
        self.motor_left_forward.ChangeDutyCycle(0)
        self.motor_left_backward.ChangeDutyCycle(0)
        self.motor_right_forward.ChangeDutyCycle(0)
        self.motor_right_backward.ChangeDutyCycle(0)

    def forward(self, speed):
        """
        Move each wheel forward
        Sets both motors to move forward at speed. 0 <= speed <= 100
        """
        self.motor_left_forward.ChangeDutyCycle(speed)
        self.motor_left_backward.ChangeDutyCycle(0)
        self.motor_right_forward.ChangeDutyCycle(speed)
        self.motor_right_backward.ChangeDutyCycle(0)

    def reverse(self, speed):
        """
        Move each wheel forward
        Sets both motors to reverse at speed. 0 <= speed <= 100
        """
        self.motor_left_forward.ChangeDutyCycle(0)
        self.motor_left_backward.ChangeDutyCycle(speed)
        self.motor_right_forward.ChangeDutyCycle(0)
        self.motor_right_backward.ChangeDutyCycle(speed)

    def spin_left(self, speed):
        """
        Causes the Robot to rotate left as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.motor_left_forward.ChangeDutyCycle(0)
        self.motor_left_backward.ChangeDutyCycle(speed)
        self.motor_right_forward.ChangeDutyCycle(speed)
        self.motor_right_backward.ChangeDutyCycle(0)

    def spin_right(self, speed):
        """
        Causes the Robot to rotate right as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.motor_left_forward.ChangeDutyCycle(speed)
        self.motor_left_backward.ChangeDutyCycle(0)
        self.motor_right_forward.ChangeDutyCycle(0)
        self.motor_right_backward.ChangeDutyCycle(speed)

    def one_wheel_right(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.turn_forward(speed, 0)

    def one_wheel_left(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.turn_forward(0, speed)

    def turn_forward(self, left_speed, right_speed):
        """
        Moves forwards in an arc by setting different speeds.
        0 <= left_speed,right_speed <= 100
        """
        self.motor_left_forward.ChangeDutyCycle(left_speed)
        self.motor_left_backward.ChangeDutyCycle(0)
        self.motor_right_forward.ChangeDutyCycle(right_speed)
        self.motor_right_backward.ChangeDutyCycle(0)

    def turn_reverse(self, left_speed, right_speed):
        """
        Moves backwards in an arc by setting different speeds.
        0 <= left_speed,right_speed <= 100
        """
        self.motor_left_forward.ChangeDutyCycle(0)
        self.motor_left_backward.ChangeDutyCycle(left_speed)
        self.motor_right_forward.ChangeDutyCycle(0)
        self.motor_right_backward.ChangeDutyCycle(right_speed)


if __name__ == "__main__":
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)
        MCONTROLLER = MotorController()
        MCONTROLLER.stop()
        time.sleep(1)
        MODULE_LOGGER.info("forward 50%")
        MCONTROLLER.forward(50)
        time.sleep(1)
        MODULE_LOGGER.info("reverse 50%")
        MCONTROLLER.reverse(50)
        time.sleep(1)
        MODULE_LOGGER.info("spin_left 50%")
        MCONTROLLER.spin_left(50)
        time.sleep(1)
        MODULE_LOGGER.info("spin_right 50%")
        MCONTROLLER.spin_right(50)
        time.sleep(1)
        MODULE_LOGGER.info("turn_forward 50%")
        MCONTROLLER.turn_forward(50, 50)
        time.sleep(1)
        MODULE_LOGGER.info("turn_reverse 50%")
        MCONTROLLER.turn_reverse(50, 50)
        time.sleep(1)
        MODULE_LOGGER.info("one_wheel_left 50%")
        MCONTROLLER.one_wheel_left(50)
        time.sleep(1)
        MODULE_LOGGER.info("one_wheel_right 50%")
        MCONTROLLER.one_wheel_right(50)
    except KeyboardInterrupt:
        pass
    finally:
        MCONTROLLER.cleanup()
