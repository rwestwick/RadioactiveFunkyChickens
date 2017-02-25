#!/usr/bin/python

"""
straightLineSpeedTest2.py needs to be run from the command line
This code is for use in the Straight Line Speed Test - PiWars 2017 challenge
http://piwars.org/
"""

# Import required libraries
import time
import logging
import math
import KeyboardCharacterReader
import MotorController
import UltrasonicSensor
import SetupConsoleLogger
import GPIOLayout

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set initial constant values

speed = MotorController.SPEED_FASTEST  # Initial forward speed
arcSpeed = 100          # Difference between left and right for correction
# Width of the wall in cm (actual width on web-page = 522 mm)
wallWidth = 52
robotWidth = 12         # Width of the robot in cm
firstBufferWidth = 20   # First steer correction distance to nearest wall
secondBufferWidth = 10  # Second steer correction distance to nearest wall
# Correction loop speed in seconds. This could be zero!
loopTime = 0.1

# Initialise motors
robotmove = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)


def wallAngle(distanceOne, distanceTwo):
    """
    Calculate angle to wall
    returns angle from front to rear line of robot to
    the wall in degrees (90deg means robot is driving
    parallel to the walls)
    N.B. Angle does not tell if robot is pointing to or away
    from wall.
    inputs:
    distanceOne - Ultrasonic measurement from one side of robot
    distanceTwo - Ultrasonic measurement from other side of robot
    N.B. Calculation is based on inputs both being in cm and walls being
    wallWidth apart.
    """
    yOne = distanceOne + (robotWidth / 2)
    yTwo = distanceTwo + (robotWidth / 2)
    hypotenuse = yOne + yTwo
    # Rounds down to nearest integer
    wallRatio = wallWidth / hyptenuse
    if (wallRatio > 1.0):
        wallRatio = 1.0
        
    theta = math.degrees(math.asin(wallRatio)))
    return theta


def distanceFromWall(distanceOne, theta):
    """
    returns distance from front to rear line of robot to
    the wall that is at theta degrees angle to wall at a
    distance of distance distanceOne.
    Unit of returned value is the same as input e.g. cm
    """
    hypotenuse = distanceOne + (robotWidth / 2)
    wallDistance = hypotenuse * math.sin(math.radians(theta))
    return wallDistance


def main():
    """
    """
    # Create necessary sensor objects
    viewLeft = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_PIN,
        GPIOLayout.SONAR_LEFT_TX_PIN)
    viewRight = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN,
        GPIOLayout.SONAR_RIGHT_TX_PIN)

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
        angleFromWall = wallAngle(leftDistance, rightDistance)
        LOGGER.info("Angle from the wall: " + str(int(angleFromWall)) + " deg")
        spaceFromLeftWall = distanceFromWall(leftDistance, angleFromWall)
        LOGGER.info("Actual distance from the left wall: " +
                    str(int(spaceFromLeftWall)) + " cm")

        # Decide which way to steer
        if (leftDistance < firstBufferWidth):
            robotmove.turn_forward(speed, (speed - arcSpeed))
            time.sleep(0.2)
            robotmove.forward(speed)
            time.sleep(0.2)
            LOGGER.info("Steering right")
        elif (rightDistance < firstBufferWidth):
            robotmove.turn_forward((speed - arcSpeed), speed)
            time.sleep(0.2)
            robotmove.forward(speed)
            time.sleep(0.2)
            LOGGER.info("Steering left")
        elif ((rightDistance < secondBufferWidth) and
              (leftDistance < secondBufferWidth)):
            robotmove.stop()
            LOGGER.info("The walls are caving in!")
        elif ((rightDistance > wallWidth) and (leftDistance > wallWidth)):
            robotmove.stop()
            LOGGER.info("I'm in the clear!")
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
