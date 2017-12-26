#!/usr/bin/python
"""
Provides the test functionality for the Ultrasonic Sensor
"""

import time
import logging
import SetupConsoleLogger
import GPIOLayout
import UltrasonicSensor

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)


def test_ultrasonic():
    try:
        PROXITY_TWO_IO_LEFT = UltrasonicSensor.UltrasonicSensor(
                                               GPIOLayout.SONAR_LEFT_RX_PIN,
                                               GPIOLayout.SONAR_LEFT_TX_PIN)
        MODULE_LOGGER.info(
            "PROXITY_TWO_IO_LEFT: " + str(PROXITY_TWO_IO_LEFT.measurement()))
        PROXITY_TWO_IO_LEFT.cleanup()

        PROXITY_TWO_IO_RIGHT = UltrasonicSensor.UltrasonicSensor(
                                                GPIOLayout.SONAR_RIGHT_RX_PIN,
                                                GPIOLayout.SONAR_RIGHT_TX_PIN)
        MODULE_LOGGER.info(
            "PROXITY_TWO_IO_RIGHT: " + str(PROXITY_TWO_IO_RIGHT.measurement()))
        PROXITY_TWO_IO_RIGHT.cleanup()

        PROXITY_ONE_IO = UltrasonicSensor.UltrasonicSensor(
                                           GPIOLayout.SONAR_FRONT_TX_PIN)
        MODULE_LOGGER.info(
            "PROXITY_ONE_IO: " + str(PROXITY_ONE_IO.measurement()))
        PROXITY_ONE_IO.cleanup()

        PROXITY_HIGH_Q = UltrasonicSensor.UltrasonicSensor(
                                               GPIOLayout.SONAR_FRONT_TX_PIN,
                                               qsize = 20)
        MODULE_LOGGER.info(
            "PROXITY_HIGH_Q: " + str(PROXITY_HIGH_Q.measurement()))
        PROXITY_HIGH_Q.cleanup()

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    test_ultrasonic()
