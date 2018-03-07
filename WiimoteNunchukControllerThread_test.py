#!/usr/bin/python
"""
Provides the test functionality for the wiimote and nunchuk controller Thread
"""

import logging
import time
import SetupConsoleLogger
import WiimoteNunchukControllerThread
import GPIOLayout

MODULE_LOGGER = logging.getLogger("__main__")
SetupConsoleLogger.setup_console_logger(MODULE_LOGGER, logging.DEBUG)

def __callback_button_b(currentWiimote):
    MODULE_LOGGER.info("__callback_button_b")
    currentWiimote.rumble = 1
    time.sleep(1)
    currentWiimote.rumble = 0
    
def __callback_button_c(currentWiimote):
    MODULE_LOGGER.info("__callback_button_c")
    currentWiimote.rumble = 1
    time.sleep(1)
    currentWiimote.rumble = 0

def __callback_button_z(currentWiimote):
    MODULE_LOGGER.info("__callback_button_z")
    currentWiimote.rumble = 1
    time.sleep(1)
    currentWiimote.rumble = 0


def test_nunchukcontroller(sleep_len=30):
    MODULE_LOGGER.info("test_nunchukcontroller")
    try:
        CONTROLLER = WiimoteNunchukControllerThread.WiimoteNunchukControllerThread(
            __callback_button_b, __callback_button_c, __callback_button_z)
        CONTROLLER.start()
        time.sleep(sleep_len)
    except KeyboardInterrupt:
        pass
    finally:
        CONTROLLER.exit_now()
        CONTROLLER.join()
        CONTROLLER.__del__()

if __name__ == "__main__":
    test_ultrasonicthread_callback(30)
