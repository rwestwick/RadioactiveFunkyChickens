#!/usr/bin/python

"""
Class to interact with ir proximity sensors
"""

import RPi.GPIO as GPIO
import logging

MODULE_LOGGER = logging.getLogger("__main__.IRSensor")

# irFL = 7
# irFR = 11
# lineLeft = 29
# lineRight = 13


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

        # use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # set up digital line detectors as inputs
        GPIO.setup(self.gpio_id, GPIO.IN)

    def ir_active(self):
        """
        Returns state of Left IR Obstacle sensor
        """
        if GPIO.input(self.gpio_id) == 0:
            return True
        else:
            return False


if __name__ == "__main__":
    try:
        SENSOR = IRSensor(7)
        print("ir_active: ", SENSOR.ir_active())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
