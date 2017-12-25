#!/usr/bin/python
"""
Class to interact with ir proximity sensors
"""

import RPi.GPIO as GPIO
import logging
import SetupConsoleLogger

MODULE_LOGGER = logging.getLogger("__main__.IRSensor")


class IRSensor(object):  # pylint: disable=too-few-public-methods
    """
    Defines the interaction with the ir proximity sensors
    """

    def __init__(self, gpio_id):
        """
        Initialise the parameters required for the IR Sensor
        """
        self.gpio_id = gpio_id
        MODULE_LOGGER.info("Setting up IRSensor Module")

        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)

        # set up digital line detectors as inputs
        GPIO.setup(self.gpio_id, GPIO.IN)

    def ir_active(self):
        """
        Returns state of Left IR Obstacle sensor
        """
        return bool(GPIO.input(self.gpio_id) == 0)


if __name__ == "__main__":
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)
        IR_PIN = 7
        SENSOR = IRSensor(IR_PIN)
        MODULE_LOGGER.info("ir_active: " + str(SENSOR.ir_active()))
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
