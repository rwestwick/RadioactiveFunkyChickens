"""
"""

import RPi.GPIO as GPIO
import logging

module_logger = logging.getLogger("__main__.IRSensor")

#irFL = 7
#irFR = 11
#lineLeft = 29
#lineRight = 13
    
class IRSensor:
    """
    """

    def __init__(self, id):
        """
        """
        self.gpio_id = id
        module_logger.info("Setting up IRSensor Module")
        
        #use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        #set up digital line detectors as inputs
        GPIO.setup(lineRight, GPIO.IN) 

    #======================================================================
    # IR Sensor Functions
    #
    # irLeft(): Returns state of Left IR Obstacle sensor
    def irActive():
        """
        """
        if GPIO.input(self.gpio_id) == 0:
            return True
        else:
            return False
        
    # End of IR Sensor Functions
    #======================================================================

    
if __name__ == "__main__":
    try:
        IRSensor sensor(7)
        print("irActive: ", irActive())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
