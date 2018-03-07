#!/usr/bin/python
"""
Class for enabling a gpio port to be switched on and off
"""

import logging
import platform
if platform.machine() == "armv6l" or platform.machine() == "armv7l":
    import RPi.GPIO as GPIO
    try:
        from gpiozero import OutputDevice
    except ImportError:
        print "ERROR importing OutputDevice from gpiozero"
else:
    from GPIOZeroStub import OutputDevice as OutputDevice
    import GPIOStub as GPIO
    
    
MODULE_LOGGER = logging.getLogger("__main__.SwitchingGPIO")


class SwitchingGPIO(object):
    """
    Defines the interaction with a GPIO socket
    """

    def __init__(self, bcm_num):
        """
        Initialise the parameters required for the switching base class
        """
        MODULE_LOGGER.info("GPIO Class init on socket " + str(bcm_num))
        
        # Use board pin numbering
        GPIO.setmode(GPIO.BCM)

        self.bcm_num = bcm_num
        self.socket = OutputDevice(bcm_num)

    def __del__(self):
        """
        Destructor
        """
        MODULE_LOGGER.info("GPIO Switch closed on socket " + str(self.bcm_num))
        self.socket.close()

    def switch_on(self):
        """
        Switches socket on
        """
        MODULE_LOGGER.debug("Switching on " + str(self.bcm_num))
        self.socket.on()

    def switch_off(self):
        """
        Switches socket off
        """
        MODULE_LOGGER.debug("Switching off " + str(self.bcm_num))
        self.socket.off()

    def is_on(self):
        """
        Gets the state of the gpio line
        """
        if self.socket.is_active:
            MODULE_LOGGER.debug("State is True")
            return True
        else:
            MODULE_LOGGER.debug("State is False")
            return False
