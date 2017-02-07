#!/usr/bin/python

"""
"""

import RPi.GPIO as GPIO
import logging

module_logger = logging.getLogger("__main__.LineFollowerSensor")


class LineFollowerSensor:

    """
    """

    def __init__(self, left_sensor_id, middle_sensor_id, right_sensor_id):
        """
        """
        module_logger.info("Setting up LineFollowerSensor Module")
        self.left_sensor_id = left_sensor_id
        self.middle_sensor_id = middle_sensor_id
        self.right_sensor_id = right_sensor_id

        # Use BCM GPIO references
        # instead of physical pin numbers
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Set pins as output and input
        GPIO.setup(self.left_sensor_id, GPIO.IN)       # Switch as input
        GPIO.setup(self.middle_sensor_id, GPIO.IN)       # Switch as input
        GPIO.setup(self.right_sensor_id, GPIO.IN)       # Switch as input

    def GetLState(self):
        """
        """
        return GPIO.input(self.left_sensor_id)

    def GetMState(self):
        """
        """
        return GPIO.input(self.middle_sensor_id)

    def GetRState(self):
        """
        """
        return GPIO.input(self.right_sensor_id)


if __name__ == "__main__":
    try:
        # Define GPIO to use on Pi
        GPIO_LINE_L = 16
        GPIO_LINE_M = 21
        GPIO_LINE_R = 20

        linefollower = LineFollowerSensor(
            GPIO_LINE_L, GPIO_LINE_M, GPIO_LINE_R)
        print("linefollower::left: ", linefollower.GetLState())
        print("linefollower::right: ", linefollower.GetRState())
        print("linefollower::middle: ", linefollower.GetMState())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
