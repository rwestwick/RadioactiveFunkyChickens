#!/usr/bin/python
"""
Provides the test functionality for the Servo Controller
"""

import time
import logging
import SetupConsoleLogger
import ServoController

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)


def test_servocontroller(sleep_len=0):
    SERVO_CONTROLLER = None

    try:
        SERVO_CONTROLLER = ServoController.ServoController()

        SERVO_CONTROLLER.start_servos()
        time.sleep(sleep_len)
        SERVO_CONTROLLER.set_pan_servo(0)
        time.sleep(sleep_len)
        SERVO_CONTROLLER.set_pan_servo(90)
        time.sleep(sleep_len)
        SERVO_CONTROLLER.set_pan_servo(-90)
        time.sleep(sleep_len)
        SERVO_CONTROLLER.set_pan_servo(0)
        time.sleep(sleep_len)

        SERVO_CONTROLLER.set_tilt_servo(0)
        time.sleep(sleep_len)
        SERVO_CONTROLLER.set_tilt_servo(90)
        time.sleep(sleep_len)
        SERVO_CONTROLLER.set_tilt_servo(-90)
        time.sleep(sleep_len)
        SERVO_CONTROLLER.set_tilt_servo(0)
    except KeyboardInterrupt:
        pass
    finally:
        SERVO_CONTROLLER.stop_servos()


if __name__ == "__main__":
    test_servocontroller(1)
