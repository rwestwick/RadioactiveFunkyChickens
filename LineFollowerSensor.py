#!/usr/bin/python

"""
Provides ability to read the three sensors of the line
follower module
"""

import logging
import RPi.GPIO as GPIO
import IRSensor
import SetupConsoleLogger
import GPIOLayout

MODULE_LOGGER = logging.getLogger("__main__.LineFollowerSensor")


class LineFollowerSensor(object):

    """
    Provides ability to read the three sensors of the line
    follower module
    """

    def __init__(self, left_sensor_id, middle_sensor_id, right_sensor_id):
        """
        Initialises the class
        """
        MODULE_LOGGER.info("Setting up LineFollowerSensor Module")

        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)

        self.sensor_l = IRSensor.IRSensor(left_sensor_id)
        self.sensor_m = IRSensor.IRSensor(middle_sensor_id)
        self.sensor_r = IRSensor.IRSensor(right_sensor_id)

    def get_l_state(self):
        """
        Get the state of the left sensor
        """
        return self.sensor_l.ir_active()

    def get_m_state(self):
        """
        Get the state of the middle sensor
        """
        return self.sensor_m.ir_active()

    def get_r_state(self):
        """
        Get the state of the right sensor
        """
        return self.sensor_r.ir_active()


if __name__ == "__main__":
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)

        LINEFOLLOWER = LineFollowerSensor(
            GPIOLayout.LINE_FOLLOWER_LEFT_PIN,
            GPIOLayout.LINE_FOLLOWER_MIDDLE_PIN,
            GPIOLayout.LINE_FOLLOWER_RIGHT_PIN)

        MODULE_LOGGER.info("LINEFOLLOWER::left: " +
                           str(LINEFOLLOWER.get_l_state()))
        MODULE_LOGGER.info("LINEFOLLOWER::right: " +
                           str(LINEFOLLOWER.get_r_state()))
        MODULE_LOGGER.info("LINEFOLLOWER::middle: " +
                           str(LINEFOLLOWER.get_m_state()))
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
