#!/usr/bin/python

"""
wallDistanceAndAngle.py
Functions:
    wallAngle(distanceOne, distanceTwo, wallWidth)
    distanceFromWall(distanceOne, theta)
For use with robot to help with wall following challenges
"""

# Import required libraries
import logging
import math
import SetupConsoleLogger

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set initial constant values
robotWidth = 12.0         # Width of the robot in cm


def wallAngle(distanceOne, distanceTwo, wallWidth):
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
    wallWidth - Shortest distance between the two measured parallel walls
    N.B. Calculation is based on inputs both being in cm and walls being
    wallWidth apart.
    """
    yOne = distanceOne + (robotWidth / 2.0)
    yTwo = distanceTwo + (robotWidth / 2.0)
    hypotenuse = yOne + yTwo
    
    # Rounds down numbers greater than one to one
    wallRatio = wallWidth / hypotenuse
    if (wallRatio > 1.0):
        wallRatio = 1.0
        
    theta = math.degrees(math.asin(wallRatio))
    return theta


def distanceFromWall(distanceOne, theta):
    """
    returns distance from front to rear line of robot to
    the wall that is at theta degrees angle to wall at a
    distance of distance distanceOne.
    Unit of returned value is the same as input e.g. cm
    """
    hypotenuse = distanceOne + (robotWidth / 2.0)
    wallDistance = hypotenuse * math.sin(math.radians(theta))
    return wallDistance


def main():
    """
    Test functions with possible mesurement values
    """
    piWarsWallWidth = 52.0 # Width between walls in cm (actual width on PiWars 2017 web-page = 522 mm)
    piWarsDistanceOne = 40.0 # Distance from one sensor to wall One
    piWarsDistanceTwo = 30.0 # Distance from one sensor to wall Two
    
    piWarsAngle = wallAngle(piWarsDistanceOne, piWarsDistanceTwo, piWarsWallWidth)
    piWarsDistanceFromWallOne = distanceFromWall(piWarsDistanceOne, piWarsAngle)
    piWarsDistanceFromWallTwo = distanceFromWall(piWarsDistanceTwo, piWarsAngle)

    LOGGER.info("Distance to first wall: " +
                str(piWarsDistanceOne) + " cm")
    LOGGER.info("Distance to second wall: " +
                str(piWarsDistanceTwo) + " cm")
    LOGGER.info("Angle of robot to walls: " +
                str(piWarsAngle) + " deg")
    LOGGER.info("Closest distance of robot to wall One: " +
                str(piWarsDistanceFromWallOne) + " cm")
    LOGGER.info("Closest distance of robot to wall Two: " +
                str(piWarsDistanceFromWallTwo) + " cm")



if __name__ == "__main__":
    main()
