#!/usr/bin/python

"""
Provides ability to read the three sensors of the line
follower module
"""

import RPi.GPIO as GPIO
import logging
import IRSensor

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
        # Define GPIO to use on Pi
        GPIO_LINE_L = 36
        GPIO_LINE_M = 40
        GPIO_LINE_R = 38

        LINEFOLLOWER = LineFollowerSensor(
            GPIO_LINE_L,
            GPIO_LINE_M,
            GPIO_LINE_R)
        print("LINEFOLLOWER::left: ", LINEFOLLOWER.get_l_state())
        print("LINEFOLLOWER::right: ", LINEFOLLOWER.get_r_state())
        print("LINEFOLLOWER::middle: ", LINEFOLLOWER.get_m_state())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
