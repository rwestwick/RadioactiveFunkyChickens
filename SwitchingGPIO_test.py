#!/usr/bin/python
"""
Provides the test functionality for the SwitchingGPIO class
"""

import logging
import SetupConsoleLogger
import SwitchingGPIO

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)


def test_switchinggpio():
    try:
        SWITCH = SwitchingGPIO.SwitchingGPIO(1)
        SWITCH.switch_on()
        SWITCH.switch_off()

    except KeyboardInterrupt:
        pass
    finally:
        pass


if __name__ == "__main__":
    test_switchinggpio()
