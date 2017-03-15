#!/usr/bin/python

"""
WiimoteController.py needs to be run from the command line
This code is for the remote-controlled PiWars 2017 challenges:
Pi Noon
Obstacle Course
Skittles
Slightly Deranged Golf
http://piwars.org/
"""

# Import required libraries
import time
import logging
import MotorController
import SetupConsoleLogger
import GPIOLayout
import cwiid

BUTTON_DELAY = 0.1 

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
robotmove = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)
    
def main():
    """
    """
    # Set initial values
    speed = MotorController.SPEED_FASTEST  # Initial forward speed
    
    # Connecting to the wiimote. This allows several attempts
    # as first few often fail.
    LOGGER.info("Press 1+2 on your Wiimote now ...")
    time.sleep(1)

    wm = None
    i=2
    while not wm:
            try:
                    wm=cwiid.Wiimote()
            except RuntimeError:
                    if (i>5):
                            LOGGER.info("Cannot create Wiimote connection.")
                            quit()
                    LOGGER.info("Error opening wiimote connection, attempt "
                                + str(i))
                    i +=1

    LOGGER.info("Wiimote connected.")
    
    # Set wiimote to report button presses
    wm.rpt_mode = cwiid.RPT_BTN

    # Turn on led to show connected
    wm.led = 1

    # Respond to key presses
    while True:
        buttons = wm.state['buttons']

        # Go forward if forward/up arrow key pressed
        if (buttons & cwiid.BTN_UP):
            robotmove.forward(speed)
            LOGGER.info("Forward at speed " + str(speed))
            time.sleep(BUTTON_DELAY)
            
        # Go backwards if 'reverse arrow key pressed
        elif (buttons & cwiid.BTN_DOWN):
            robotmove.reverse(speed)
            LOGGER.info("Reverse at speed " + str(speed))
            time.sleep(BUTTON_DELAY)

        # Spin right right arrow key pressed
        elif (buttons & cwiid.RIGHT):
            robotmove.spin_right(speed)
            LOGGER.info("Spin right at speed " + str(speed))
            time.sleep(BUTTON_DELAY)
            
        # Spin left if left arrow key pressed
        elif (buttons & cwiid.BTN_LEFT):
            robotmove.spin_left(speed)
            LOGGER.info("Spin left at speed " + str(speed))
            time.sleep(BUTTON_DELAY)

        # If botton A pressed stop and rumble
        elif (button & cwiid.BTN_A):
            robotmove.stop()
            wii.rumble = 1
            time.sleep(1)
            wii.rumble = 0
            LOGGER.info("Stop!")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the Wiimote Controller")
    finally:
        LOGGER.info("Wiimote Controller Finished")
        robotmove.cleanup()
