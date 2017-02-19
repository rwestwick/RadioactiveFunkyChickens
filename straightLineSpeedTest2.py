#!/usr/bin/python

"""
straightLineSpeedTest2.py needs to be run from the command line
This code is for use in the Straight Line Speed Test - PiWars 2017 challenge
http://piwars.org/
"""

# Import required libraries
import time
import logging
import math # https://docs.python.org/2/library/math.html
import KeyboardCharacterReader
import MotorController
import UltrasonicSensor

# Create a logger to both file and stdout
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
formatter = logging.Formatter('%(asctime)s - %(name)25s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)

# Set initial constant values

speed = 60                # Initial forward speed
arcSpeed = 20             # Difference between left and right for correction
wallWidth = 52            # Width of the wall in cm (actual width on web-page = 522 mm)
robotWidth = 12           # Width of the robot in cm
firstBufferWidth = 20     # First steer correction distance to nearest wall
secondBufferWidth = 10    # Second steer correction distance to nearest wall
loopTime = 2.0            # Correction loop speed in seconds. This could be zero!


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
    N.B. Calculation is based on inputs both being in cm and walls being wallWidth apart.
    """
    yOne = distanceOne + (robotWidth/2)
    yTwo = distanceTwo + (robotWidth/2)
    hypotenuse = yOne + yTwo
    theta = math.degrees(math.asin(wallWidth/hypotenuse))
    return theta


def distanceFromWall(distanceOne, theta):
    """
    returns distance from front to rear line of robot to
    the wall that is at theta degrees angle to wall at a
    distance of distance distanceOne.
    Unit of returned value is the same as input e.g. cm
    """
    hypotenuse = distanceOne + (robotWidth/2)
    wallDistance = hypotenuse * math.sin(math.radians(theta))
    return wallDistance


# Create necessary sensor objects
SONAR_INPUT_RIGHT = 16  # Connected to Echo
SONAR_OUTPUT_RIGHT = 31  # Connected to Trig
SONAR_INPUT_LEFT = 15  # Connected to Echo
SONAR_OUTPUT_LEFT = 12  # Connected to Trig
viewLeft = UltrasonicSensor.UltrasonicSensor(SONAR_INPUT_LEFT, SONAR_OUTPUT_LEFT)
viewRight = UltrasonicSensor.UltrasonicSensor(SONAR_INPUT_RIGHT, SONAR_OUTPUT_RIGHT)

time.sleep(1)

# Sanity check the distance from edge of robot to walls
initialLeftDistance = viewLeft.measurement()
initialRightDistance = viewRight.measurement()

if ((initialLeftDistance + initialRightDistance + robotWidth) > (wallWidth * 1.2)):
    logger.warn("The walls are too far apart!")
else:
    logger.info("The walls are not too far apart!")

# Waiting for start of race
logger.info("To start race press 'Space' key.")

while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            logger.info("Go")
            break

# Initialise motors
robotmove = MotorController.MotorController()

# Drive forward at full speed
robotmove.forward(speed)

logger.info("They're off! Press Control^C to finish")

# Try to avoid the walls!
try:
    while True:
        # Calculate the distance from edge of robot to walls
        leftDistance = viewLeft.measurement()
        logger.info("Average measured left distance: " + str(int(leftDistance)) + " cm")
        rightDistance = viewRight.measurement()
        logger.info("Average measured right distance: " + str(int(rightDistance)) + " cm")
        angleFromWall = wallAngle(leftDistance, rightDistance)
        logger.info("Angle from the wall: " + str(int(angleFromWall)) + " deg")
        spaceFromLeftWall = distanceFromWall(leftDistance, angleFromWall)
        logger.info("Actual distance from the left wall: " + str(int(spaceFromLeftWall)) + " cm")

        # Decide which way to steer
        if (leftDistance < firstBufferWidth):
            robotmove.turn_forward(speed, (speed - arcSpeed))
            logger.info("Steering right")
        elif (rightDistance < firstBufferWidth):
            robotmove.turn_forward((speed - arcSpeed), speed)
            logger.info("Steering left")
        elif ((rightDistance < secondBufferWidth) and (leftDistance < secondBufferWidth)):
            robotmove.stop()
            logger.info("The walls are caving in!")
        elif ((rightDistance > wallWidth) and (leftDistance > wallWidth)):
            robotmove.stop()
            logger.info("I'm in the clear!")
        else:
            robotmove.forward(speed)
            logger.info("Storming forward!")
        
        # Loop delay
        time.sleep(loopTime)
        
except KeyboardInterrupt:
    logger.info("Stopping the race")

# Sets all motors off and sets GPIO to standard values
finally:
    robotmove.cleanup()
