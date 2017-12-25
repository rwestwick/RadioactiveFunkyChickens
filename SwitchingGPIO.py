#!/usr/bin/python
"""
Class for enabling a gpio port to be switched on and off
"""

import logging
import SetupConsoleLogger
MODULE_LOGGER = logging.getLogger("__main__.SwitchingGPIO")

try:
    from gpiozero import GPIODevice
except ImportError:
    MODULE_LOGGER.error("ERROR importing GPIODevice from gpiozero")


class SwitchingGPIO(object):
    """
    Defines the interaction with a GPIO socket
    """

    def __init__(self, pcm_num):
        """
        Initialise the parameters required for the switching base class
        """
        MODULE_LOGGER.info("GPIO Class init on socket" + str(pcm_num))
        self.pcm_num = pcm_num
        self.socket = GPIODevice(pcm_num)

    def __del__(self):
        """
        Destructor
        """
        self.socket.close()

    def switch_on(self):
        """
        Switches socket on
        """
        MODULE_LOGGER.info("Switching on " + str(self.pcm_num))
        self.socket.on()

    def switch_off(self):
        """
        Switches socket off
        """
        MODULE_LOGGER.info("Switching off " + str(self.pcm_num))
        self.socket.off()


if __name__ == "__main__":
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)
        SWITCH = SwitchingGPIO(1)
        SWITCH.switch_on()
        SWITCH.switch_off()
    except KeyboardInterrupt:
        pass
    finally:
        pass
