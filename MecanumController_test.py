#!/usr/bin/python
"""
Provides the test functionality for the MecanumController
"""

import time
import logging
import SetupConsoleLogger
import GPIOLayout
import SpeedSettings
import MecanumController

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)


def test_mecanumcontroller(sleep_len=0):
    MMCONTROLLER = None
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)
        MMCONTROLLER = MecanumController.MecanumController(
            GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)
        MMCONTROLLER.stop()
        time.sleep(sleep_len)
        MODULE_LOGGER.info("forward 50%")
        MMCONTROLLER.forward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("backward 50%")
        MMCONTROLLER.backward(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("spin_left 50%")
        MMCONTROLLER.spin_left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("spin_right 50%")
        MMCONTROLLER.spin_right(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("left 50%")
        MMCONTROLLER.left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("right 50%")
        MMCONTROLLER.right(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("forward_diagonal_left 50%")
        MMCONTROLLER.forward_diagonal_left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("forward_diagonal_right 50%")
        MMCONTROLLER.forward_diagonal_right(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("backward_diagonal_left 50%")
        MMCONTROLLER.backward_diagonal_left(SpeedSettings.SPEED_MEDIUM)
        time.sleep(sleep_len)
        MODULE_LOGGER.info("backward_diagonal_right 50%")
        MMCONTROLLER.backward_diagonal_right(SpeedSettings.SPEED_MEDIUM)

    except KeyboardInterrupt:
        pass
    finally:
        MMCONTROLLER.cleanup()


if __name__ == "__main__":
    test_mecanumcontroller(2)
