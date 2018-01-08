#!/usr/bin/python
"""
Provides the test functionality for the IRSensor
"""

import logging
import SetupConsoleLogger
import GPIOLayout
import IRSensor

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)


def test_irsensor():
    SENSOR = None

    try:
        SENSOR = IRSensor.IRSensor(GPIOLayout.LINE_FOLLOWER_MIDDLE_PIN)
        MODULE_LOGGER.info("ir_active: " + str(SENSOR.ir_active()))
    except KeyboardInterrupt:
        pass
    finally:
        SENSOR.cleanup()


if __name__ == "__main__":
    test_irsensor()
