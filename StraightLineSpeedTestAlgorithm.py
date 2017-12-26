#!/usr/bin/python
"""
straightLineSpeedTest2.py needs to be run from the command line
This code is for use in the Straight Line Speed Test - PiWars 2017 challenge
http://piwars.org/
"""

# Import required libraries
import time
import logging
import KeyboardCharacterReader
import MotorController
import UltrasonicSensor
import SetupConsoleLogger
import GPIOLayout
import SpeedSettings

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)
# Set initial constant values

speed = SpeedSettings.SPEED_FASTEST  # Initial forward speed
arcSpeed = 100  # Difference between left and right for correction
# Width of the wall in cm (actual width on web-page = 522 mm)
wallWidth = 52
robotWidth = 12  # Width of the robot in cm
firstBufferWidth = 20  # First steer correction distance to nearest wall
secondBufferWidth = 10  # Second steer correction distance to nearest wall
# Correction loop speed in seconds. This could be zero!
loopTime = 0.1
# Angle correction delay time in seconds
correctionTime = 0.2

# Initialise motors
robotmove = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN, GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN, GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)


def main():
    """
    """
    # Create necessary sensor objects
    viewLeft = UltrasonicSensor.UltrasonicSensor(GPIOLayout.SONAR_LEFT_RX_PIN,
                                                 GPIOLayout.SONAR_LEFT_TX_PIN)
    viewRight = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN, GPIOLayout.SONAR_RIGHT_TX_PIN)

    # Sanity check the distance from edge of robot to walls
    initialLeftDistance = viewLeft.measurement()
    initialRightDistance = viewRight.measurement()

    if ((initialLeftDistance + initialRightDistance + robotWidth) >
            (wallWidth * 1.2)):
        LOGGER.warn("The walls are too far apart!")
    else:
        LOGGER.info("The walls are not too far apart!")

    # Waiting for start of race
    LOGGER.info("To start race press 'Space' key.")

    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            LOGGER.info("They're off! Press Control^C to finish")
            break

    # Drive forward at full speed in order to find the line.
    robotmove.forward(speed)

    # Try to avoid the walls!
    while True:
        # Calculate the distance from edge of robot to walls
        leftDistance = viewLeft.measurement()
        LOGGER.info("Average measured left distance: " +
                    str(int(leftDistance)) + " cm")

        rightDistance = viewRight.measurement()
        LOGGER.info("Average measured right distance: " +
                    str(int(rightDistance)) + " cm")

        # Decide which way to steer
        if (leftDistance < firstBufferWidth):
            LOGGER.info("Steering right" + str((speed - arcSpeed)))
            robotmove.turn_forward(speed, (speed - arcSpeed))
            time.sleep(correctionTime)
            robotmove.forward(speed)
            time.sleep(correctionTime)
        elif (rightDistance < firstBufferWidth):
            LOGGER.info("Steering left" + str((speed - arcSpeed)))
            robotmove.turn_forward((speed - arcSpeed), speed)
            time.sleep(correctionTime)
            robotmove.forward(speed)
            time.sleep(correctionTime)
        else:
            robotmove.forward(speed)
            LOGGER.info("Storming forward!")

        # Loop delay
        time.sleep(loopTime)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the Straight Line Test")
    finally:
        LOGGER.info("Straight Line Test Finished")
        robotmove.cleanup()
