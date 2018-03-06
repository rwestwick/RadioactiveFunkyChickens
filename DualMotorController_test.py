#!/usr/bin/python
"""
Provides the test functionality for the DualMotorController
"""

import time
import logging
import SetupConsoleLogger
import GPIOLayout
import SpeedSettings
import DualMotorController

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)


def test_motorcontroller(sleep_len=0):
    DMCONTROLLER = None
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)
        DMCONTROLLER = DualMotorController.DualMotorController(
            GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)
        DMCONTROLLER.stop()
        time.sleep(sleep_len)
        MODULE_LOGGER.info("forward 50%")
        DMCONTROLLER.forward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("reverse 50%")
        DMCONTROLLER.reverse(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("spin_left 50%")
        DMCONTROLLER.spin_left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("spin_right 50%")
        DMCONTROLLER.spin_right(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("front_left_forward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.front_left_forward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("front_left_backward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.front_left_backward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("front_right_forward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.front_right_forward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("front_right_backward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.front_right_backward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("rear_left_forward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.rear_left_forward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("rear_left_backward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.rear_left_backward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("rear_right_forward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.rear_right_forward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("rear_right_backward 50%")
        DMCONTROLLER.stop()
        DMCONTROLLER.rear_right_backward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        DMCONTROLLER.turn_forward(SpeedSettings.SPEED_FASTEST,
                                  SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        DMCONTROLLER.turn_reverse(SpeedSettings.SPEED_FASTEST,
                                  SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)

    except KeyboardInterrupt:
        pass
    finally:
        DMCONTROLLER.cleanup()


if __name__ == "__main__":
    test_motorcontroller(2)
