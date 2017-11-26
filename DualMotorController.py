#!/usr/bin/python

"""
Provides ability to control the motors using two controllers on the robot.
"""

import time
import logging
import RPi.GPIO as GPIO
import SetupConsoleLogger
import GPIOLayout
import MotorController

MODULE_LOGGER = logging.getLogger("__main__.DualMotorController")

# Define some basic speed settings
SPEED_FASTEST = 100
SPEED_FAST = 90
SPEED_MEDIUM = 80
SPEED_SLOW = 60
SPEED_VERYSLOW = 50
SPEED_VERYVERYSLOW = 40

class DualMotorController(object):

    """
    Provides ability to control the motors on the robot.
    """
    START_FREQ = 25

    def __init__(self,
                 left_front_forward,
                 left_front_backward,
                 right_front_forward,
                 right_front_backward,
                 left_back_forward,
                 left_back_backward,
                 right_back_forward,
                 right_back_backward):
                 
        MODULE_LOGGER.info("Initialising DualMotorController")

        self.front_controller = MotorController.MotorController(left_front_forward,
                                                left_front_backward,
                                                right_front_forward,
                                                right_front_backward)

        self.rear_controller = MotorController.MotorController(left_back_forward,
                                               left_back_backward,
                                               right_back_forward,
                                               right_back_backward)


    def cleanup(self):
        """
        Sets all motors off and sets GPIO to standard values
        """
        self.front_controller.stop()
        self.rear_controller.stop()
        time.sleep(1)
        GPIO.cleanup()
        MODULE_LOGGER.info("Cleaned up DualMotorController")

    def stop(self):
        """
        Causes the Robot to stop all motors
        """
        self.front_controller.stop()
        self.rear_controller.stop()

    def forward(self, speed):
        """
        Move each wheel forward
        Sets both motors to move forward at speed. 0 <= speed <= 100
        """
        self.front_controller.forward(speed)
        self.rear_controller.forward(speed)

    def reverse(self, speed):
        """
        Move each wheel forward
        Sets both motors to reverse at speed. 0 <= speed <= 100
        """
        self.front_controller.reverse(speed)
        self.rear_controller.reverse(speed)

    def spin_left(self, speed):
        """
        Causes the Robot to rotate left as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.front_controller.spin_left(speed)
        self.rear_controller.spin_left(speed)

    def spin_right(self, speed):
        """
        Causes the Robot to rotate right as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.front_controller.spin_right(speed)
        self.rear_controller.spin_right(speed)

    def front_left_forward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.using_left_go_right(speed)
        self.rear_controller.stop()

    def front_left_backward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.using_left_go_left(speed)
        self.rear_controller.stop()

    def front_right_forward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.using_right_go_right(speed)
        self.rear_controller.stop()

    def front_right_backward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.using_right_go_left(speed)
        self.rear_controller.stop()

    def rear_left_forward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.stop()
        self.rear_controller.using_left_go_right(speed)

    def rear_left_backward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.stop()
        self.rear_controller.using_left_go_left(speed)

    def rear_right_forward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.stop()
        self.rear_controller.using_right_go_right(speed)

    def rear_right_backward(self, speed):
        """
        Causes the Robot to turn right using just one wheel
        Sets just one side to turn. 0 <= speed <= 100
        """
        self.front_controller.stop()
        self.rear_controller.using_right_go_left(speed)



if __name__ == "__main__":
    DMCONTROLLER = None
    SLEEP_COUNT = 2
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)
        DMCONTROLLER = DualMotorController(
                        GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_PIN,
                        GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_PIN,
                        GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_PIN,
                        GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_PIN,
                        GPIOLayout.MOTOR_LEFT_REAR_FORWARD_PIN,
                        GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_PIN,
                        GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_PIN,
                        GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_PIN)
        DMCONTROLLER.stop()
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("forward 50%")
        DMCONTROLLER.forward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("reverse 50%")
        DMCONTROLLER.reverse(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("spin_left 50%")
        DMCONTROLLER.spin_left(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("spin_right 50%")
        DMCONTROLLER.spin_right(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("front_left_forward 50%")
        DMCONTROLLER.front_left_forward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("front_left_backward 50%")
        DMCONTROLLER.front_left_backward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("front_right_forward 50%")
        DMCONTROLLER.front_right_forward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("front_right_backward 50%")
        DMCONTROLLER.front_right_backward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("rear_left_forward 50%")
        DMCONTROLLER.rear_left_forward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("rear_left_backward 50%")
        DMCONTROLLER.rear_left_backward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("rear_right_forward 50%")
        DMCONTROLLER.rear_right_forward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("rear_right_backward 50%")
        DMCONTROLLER.rear_right_backward(SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)

    except KeyboardInterrupt:
        pass
    finally:
        DMCONTROLLER.cleanup()
