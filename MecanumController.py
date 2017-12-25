#!/usr/bin/python
"""
Provides ability to control the robots Mecanum wheels
"""

import time
import logging
import SetupConsoleLogger
import GPIOLayout
import DualMotorController
import SpeedSettings

MODULE_LOGGER = logging.getLogger("__main__.MecanumController")


class MecanumController(object):
    """
    Provides ability to control the Mecanum wheeled robot.
    """

    def __init__(self, left_front_forward, left_front_backward,
                 right_front_forward, right_front_backward, left_back_forward,
                 left_back_backward, right_back_forward, right_back_backward):

        MODULE_LOGGER.info("Initialising MecanumController")

        self.controller = DualMotorController.DualMotorController(
            left_front_forward, left_front_backward, right_front_forward,
            right_front_backward, left_back_forward, left_back_backward,
            right_back_forward, right_back_backward)

    def cleanup(self):
        """
        Sets all motors off and sets GPIO to standard values
        """
        self.controller.cleanup()
        MODULE_LOGGER.info("Cleaned up MecanumController")

    def stop(self):
        """
        Causes the Robot to stop all motors
        """
        self.controller.stop()

    def forward(self, speed):
        """
        Move each wheel forward
        Sets both motors to move forward at speed. 0 <= speed <= 100
        """
        self.controller.forward(speed)

    def backward(self, speed):
        """
        Move each wheel forward
        Sets both motors to reverse at speed. 0 <= speed <= 100
        """
        self.controller.reverse(speed)

    def spin_left(self, speed):
        """
        Causes the Robot to rotate left as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.controller.spin_left(speed)

    def spin_right(self, speed):
        """
        Causes the Robot to rotate right as fast as possible
        Sets motors to turn opposite directions at speed. 0 <= speed <= 100
        """
        self.controller.spin_right(speed)

    def left(self, speed):
        """
        Causes the Robot to move directly to the left
        speed. 0 <= speed <= 100
        """
        self.controller.front_left_backward(speed)
        self.controller.front_right_forward(speed)
        self.controller.rear_left_forward(speed)
        self.controller.rear_left_backward(speed)

    def right(self, speed):
        """
        Causes the Robot to move directly to the right
        speed. 0 <= speed <= 100
        """
        self.controller.front_left_forward(speed)
        self.controller.front_right_backward(speed)
        self.controller.rear_left_backward(speed)
        self.controller.rear_right_forward(speed)

    def forward_diagonal_left(self, speed):
        """
        Causes the Robot to move diagonally forward left
        speed. 0 <= speed <= 100
        """
        self.stop()
        self.controller.front_right_forward(speed)
        self.controller.rear_left_forward(speed)

    def forward_diagonal_right(self, speed):
        """
        Causes the Robot to move diagonally forward right
        speed. 0 <= speed <= 100
        """
        self.stop()
        self.controller.front_left_forward(speed)
        self.controller.rear_right_forward(speed)

    def backward_diagonal_left(self, speed):
        """
        Causes the Robot to move diagonally backward left
        speed. 0 <= speed <= 100
        """
        self.stop()
        self.controller.front_left_backward(speed)
        self.controller.rear_right_backward(speed)

    def backward_diagonal_right(self, speed):
        """
        Causes the Robot to move diagonally backward right
        speed. 0 <= speed <= 100
        """
        self.stop()
        self.controller.front_right_backward(speed)
        self.controller.rear_left_backward(speed)


if __name__ == "__main__":
    MMCONTROLLER = None
    SLEEP_COUNT = 2
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)
        MMCONTROLLER = MecanumController(
            GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_PIN,
            GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_PIN,
            GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_PIN,
            GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_PIN,
            GPIOLayout.MOTOR_LEFT_REAR_FORWARD_PIN,
            GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_PIN,
            GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_PIN,
            GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_PIN)
        MMCONTROLLER.stop()
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("forward 50%")
        MMCONTROLLER.forward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("backward 50%")
        MMCONTROLLER.backward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("spin_left 50%")
        MMCONTROLLER.spin_left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("spin_right 50%")
        MMCONTROLLER.spin_right(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("left 50%")
        MMCONTROLLER.left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("right 50%")
        MMCONTROLLER.right(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("forward_diagonal_left 50%")
        MMCONTROLLER.forward_diagonal_left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("forward_diagonal_right 50%")
        MMCONTROLLER.forward_diagonal_right(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("backward_diagonal_left 50%")
        MMCONTROLLER.backward_diagonal_left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(SLEEP_COUNT)
        MODULE_LOGGER.info("backward_diagonal_right 50%")
        MMCONTROLLER.backward_diagonal_right(SpeedSettings.SPEED_MEDIUM)

    except KeyboardInterrupt:
        pass
    finally:
        MMCONTROLLER.cleanup()
