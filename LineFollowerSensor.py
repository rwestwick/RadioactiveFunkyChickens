"""
"""

import RPi.GPIO as GPIO
import logging

module_logger = logging.getLogger("__main__.LineFollowerSensor")

class LineFollowerSensor:
    """
    """
    
    # Define GPIO to use on Pi
    GPIO_LINE_L = 16
    GPIO_LINE_M = 21
    GPIO_LINE_R = 20
    
    def __init__(self):
        """
        """
        module_logger.info("Setting up LineFollowerSensor Module")
        
        # Use BCM GPIO references
        # instead of physical pin numbers
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Set pins as output and input
        GPIO.setup(self.GPIO_LINE_L, GPIO.IN)       # Switch as input
        GPIO.setup(self.GPIO_LINE_M, GPIO.IN)       # Switch as input
        GPIO.setup(self.GPIO_LINE_R, GPIO.IN)       # Switch as input

    def GetLState(self):
        """
        """
        return GPIO.input(self.GPIO_LINE_L)

    def GetMState(self):
        """
        """
        return GPIO.input(self.GPIO_LINE_M)

    def GetRState(self):
        """
        """
        return GPIO.input(self.GPIO_LINE_R)

    
if __name__ == "__main__":
    try:
        LineFollowerSensor linefollower
        print("linefollower::left: ", linefollower.GetLState())
        print("linefollower::right: ", linefollower.GetRState())
        print("linefollower::middle: ", linefollower.GetMState())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
