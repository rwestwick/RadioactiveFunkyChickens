#!/usr/bin/python
"""
Class for enabling a gpio port to be switched on and off
"""

import logging
import platform
if platform.machine() == "armv6l" or platform.machine() == "armv7l":
    try:
        from gpiozero import GPIODevice
    except ImportError:
        print "ERROR importing GPIODevice from gpiozero"
else:
    from GPIOZeroStub import GPIODevice as GPIODevice

MODULE_LOGGER = logging.getLogger("__main__.SwitchingGPIO")


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
