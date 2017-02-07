#!/usr/bin/python

"""
"""

import RPi.GPIO as GPIO
import logging

module_logger = logging.getLogger("__main__.IRSensor")

# irFL = 7
# irFR = 11
# lineLeft = 29
# lineRight = 13


class IRSensor:

    """
    """

    def __init__(self, id):
        """
        Initialise the parameters required for the IR Sensor
        """
        self.gpio_id = id
        module_logger.info("Setting up IRSensor Module")

        # use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # set up digital line detectors as inputs
        GPIO.setup(self.gpio_id, GPIO.IN)

    def irActive(self):
        """
        Returns state of Left IR Obstacle sensor
        """
        if GPIO.input(self.gpio_id) == 0:
            return True
        else:
            return False


if __name__ == "__main__":
    try:
        sensor = IRSensor(7)
        print("irActive: ", sensor.irActive())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
