#!/usr/bin/python

"""
"""

import RPi.GPIO as GPIO
import time
import logging

module_logger = logging.getLogger("__main__.MotorController")


class MotorController:

    """
    """

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
        module_logger.info("Setting up MotorController Module")

        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)

        # use pwm on inputs so motors don't go too fast
        GPIO.setup(self.LEFT_FORWARD, GPIO.OUT)
        self.p = GPIO.PWM(self.LEFT_FORWARD, self.START_FREQ)
        self.p.start(0)

        GPIO.setup(self.LEFT_BACKWARD, GPIO.OUT)
        self.q = GPIO.PWM(self.LEFT_BACKWARD, self.START_FREQ)
        self.q.start(0)

        GPIO.setup(self.RIGHT_FORWARD, GPIO.OUT)
        self.a = GPIO.PWM(self.RIGHT_FORWARD, self.START_FREQ)
        self.a.start(0)

        GPIO.setup(self.RIGHT_BACKWARD, GPIO.OUT)
        self.b = GPIO.PWM(self.RIGHT_BACKWARD, self.START_FREQ)
        self.b.start(0)

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
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(0)

    def forward(self, speed):
        """
        Move each wheel forward
        Sets both motors to move forward at speed. 0 <= speed <= 100
        """
        self.p.ChangeDutyCycle(speed)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(speed)
        self.b.ChangeDutyCycle(0)

    def reverse(self, speed):
        """
        Move each wheel forward
        Sets both motors to reverse at speed. 0 <= speed <= 100
        """
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(speed)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(speed)

    def spinLeft(self, speed):
        """
        Causes the Robot to rotate left as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(speed)
        self.a.ChangeDutyCycle(speed)
        self.b.ChangeDutyCycle(0)

    def spinRight(self, speed):
        """
        Causes the Robot to rotate right as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.p.ChangeDutyCycle(speed)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(speed)

    def oneWheelRight(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.turnForward(speed, 0)

    def oneWheelLeft(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.turnForward(0, speed)

    def turnForward(self, leftSpeed, rightSpeed):
        """
        Moves forwards in an arc by setting different speeds.
        0 <= leftSpeed,rightSpeed <= 100
        """
        self.p.ChangeDutyCycle(leftSpeed)
        self.q.ChangeDutyCycle(0)
        self.a.ChangeDutyCycle(rightSpeed)
        self.b.ChangeDutyCycle(0)

    def turnReverse(self, leftSpeed, rightSpeed):
        """
        Moves backwards in an arc by setting different speeds.
        0 <= leftSpeed,rightSpeed <= 100
        """
        self.p.ChangeDutyCycle(0)
        self.q.ChangeDutyCycle(leftSpeed)
        self.a.ChangeDutyCycle(0)
        self.b.ChangeDutyCycle(rightSpeed)


if __name__ == "__main__":
    try:
        mcontroller = MotorController()
        mcontroller.stop()
        time.sleep(1)
        mcontroller.forward(50)
        time.sleep(1)
        mcontroller.reverse(50)
        time.sleep(1)
        mcontroller.spinLeft(50)
        time.sleep(1)
        mcontroller.spinRight(50)
        time.sleep(1)
        mcontroller.turnForward(50, 50)
        time.sleep(1)
        mcontroller.turnReverse(50, 50)
        time.sleep(1)
        mcontroller.oneWheelLeft(50)
        time.sleep(1)
        mcontroller.oneWheelRight(50)

    except KeyboardInterrupt:
        pass
    finally:
        mcontroller.cleanup()
