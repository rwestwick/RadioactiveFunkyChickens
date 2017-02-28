#!/usr/bin/python

"""
remoteController.py needs to be run from the command line
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
import KeyboardCharacterReader
import MotorController
import SetupConsoleLogger
import GPIOLayout

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
robotmove = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

def changeSpeed(currentDirection, newSpeed):
    """
    """
    if currentDirection == 'forward':
        robotmove.forward(newSpeed)
    elif currentDirection == 'reverse':
        robotmove.reverse(newSpeed)
    elif currentDirection == 'spin_right':
        robotmove.spin_right(newSpeed)
    elif currentDirection == 'spin_left':
        robotmove.spin_left(newSpeed)
    else:
        LOGGER.info("Incorrect currentDirection = " + str(currentDirection))
    
def main():
    """
    """
    # Set initial values
    speed = MotorController.SPEED_FASTEST  # Initial forward speed
    direction = ' '
    
    # Give remote control keys
    print "Steer robot by using the arrow keys to control"
    print "Use , or < to slow down"
    print "Use . or > to speed up"
    print "Speed changes take effect when the next arrow key is pressed"
    print "Press Ctrl-C to end"
    
    # Waiting for start of remote controller
    LOGGER.info("To start remote controller press 'Space' key.")

    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("They're off! Press Control^C to finish")
            break

    # Respond to key presses
    while True:
        keyp = KeyboardCharacterReader.readkey()

        # Go forward if 'w' or forward arrow key pressed
        if keyp == 'w' or ord(keyp) == 16:
            robotmove.forward(speed)
            direction = 'forward'
            LOGGER.info("Forward at speed " + str(speed))
            
        # Go backwards if 'z' or reverse arrow key pressed
        elif keyp == 'z' or ord(keyp) == 17:
            robotmove.reverse(speed)
            direction = 'reverse'
            LOGGER.info("Reverse at speed " + str(speed))

        # Spin right if 's' or right arrow key pressed
        elif keyp == 's' or ord(keyp) == 18:
            robotmove.spin_right(speed)
            direction = 'spin_right'
            LOGGER.info("Spin right at speed " + str(speed))
            
        # Spin left if 'a' or left arrow key pressed
        elif keyp == 'a' or ord(keyp) == 19:
            robotmove.spin_left(speed)
            direction = 'spin_left'
            LOGGER.info("Spin left at speed " + str(speed))
            
        # Speed up by 10 if '.' or '>' key pressed
        elif keyp == '.' or keyp == '>':
            speed = min(100, speed + 10)
            changeSpeed(direction, speed)
            LOGGER.info("Speed increased to " + str(speed))

        # Speed down by 10 if ',' or '<' key pressed
        elif keyp == ',' or keyp == '<':
            speed = max(0, speed - 10)
            changeSpeed(direction, speed)
            LOGGER.info("Speed decreased to " + str(speed))

        # Stop if Space (' ') key pressed    
        elif keyp == ' ':
            robotmove.stop()
            LOGGER.info("Stop!")

        # To end if Ctrl-C pressed
        elif ord(keyp) == 3:
            LOGGER.info("Break control loop!")
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the Remote Controller")
    finally:
        LOGGER.info("Remote Controller Finished")
        robotmove.cleanup()
