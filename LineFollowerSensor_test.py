#!/usr/bin/python
"""
Provides the test functionality for the LineFollowerSensor
"""

import logging
import SetupConsoleLogger
import GPIOLayout
import LineFollowerSensor

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)


def test_linefollowersensor():
    LINEFOLLOWER = None

    try:
        LINEFOLLOWER = LineFollowerSensor.LineFollowerSensor(
            GPIOLayout.LINE_FOLLOWER_LEFT_PIN,
            GPIOLayout.LINE_FOLLOWER_MIDDLE_PIN,
            GPIOLayout.LINE_FOLLOWER_RIGHT_PIN)

        MODULE_LOGGER.info(
            str(LINEFOLLOWER.get_r_state()) + " - " +
            str(LINEFOLLOWER.get_m_state()) + " - " +
            str(LINEFOLLOWER.get_l_state()))

        MODULE_LOGGER.info(
            "LINEFOLLOWER::left: " + str(LINEFOLLOWER.get_l_state()))
        MODULE_LOGGER.info(
            "LINEFOLLOWER::middle: " + str(LINEFOLLOWER.get_m_state()))
        MODULE_LOGGER.info(
            "LINEFOLLOWER::right: " + str(LINEFOLLOWER.get_r_state()))
    except KeyboardInterrupt:
        pass
    finally:
        LINEFOLLOWER.cleanup()


if __name__ == "__main__":
    test_linefollowersensor()