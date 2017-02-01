#!/usr/bin/python
#
# straightLineSpeedTest2.py needs to be run from the command line
#
# This code is for use in the Straight Line Speed Test - PiWars 2017 challenge
# http://piwars.org/
#
#======================================================================

# Import required libraries
import sonarRFC
import time

# Set initial constant values

speed = 60                # Initial forward speed
arcSpeed = 20             # Difference between left and right for correction
wallWidth = 52            # Width of the wall in cm (actual width on web-page = 522 mm)
robotWidth = 12           # Width of the robot in cm
firstBufferWidth = 20     # First steer correction distance to nearest wall
secondBufferWidth = 10    # Second steer correction distance to nearest wall
loopTime = 2.0            # Correction loop speed in seconds. This could be zero!

##########################
# Define local functions #
##########################

#======================================================================
# Reading single character by forcing stdin to raw mode
import sys
import tty
import termios

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows

# End of single character reading
#======================================================================

#======================================================================
# Calaculate angle to wall
import math # https://docs.python.org/2/library/math.html

# wallAngle(distanceOne, distanceTwo): returns angle from front to rear line of robot to
#                                      the wall in degrees (90deg means robot is driving
#                                      parallel to the walls)
#                                      N.B. Angle does not tell if robot is pointing to or away
#                                      from wall.
#                                      inputs:
#                                       distanceOne - Ultrasonic measurement from one side of robot
#                                       distanceTwo - Ultrasonic measurement from other side of robot
# N.B. Calculation is based on inputs both being in cm and walls being wallWidth apart.

def wallAngle(distanceOne, distanceTwo):
    yOne = distanceOne + (robotWidth/2)
    yTwo = distanceTwo + (robotWidth/2)

    hypotenuse = yOne + yTwo

    theta = math.degrees(math.asin(wallWidth/hypotenuse))
    
    return theta

# distanceFromWall(distanceOne, theta): returns distance from front to rear line of robot to
#                                       the wall that is at theta degrees angle to wall at a
#                                       distance of distance distanceOne.
#                                       Unit of returned value is the same as input e.g. cm

def distanceFromWall(distanceOne, theta):
    hypotenuse = distanceOne + (robotWidth/2)

    wallDistance = hypotenuse * math.sin(math.radians(theta))

    return wallDistance

# End of angle to wall calculations
#======================================================================

##########################
# Main program           #
##########################

# Initialise motors
sonarRFC.initMotors()

# Create necessary sensor objects
viewLeft = sonarRFC.leftUltraSensor()
viewRight = sonarRFC.rightUltraSensor()

time.sleep(1)

# Sanity check the distance from edge of robot to walls
initialLeftDistance = viewLeft.Measurement()
initialRightDistance = viewRight.Measurement()

if ((initialLeftDistance + initialRightDistance + robotWidth) > (wallWidth * 1.2)):
    print("The walls are too far apart!")
else:
    print("The walls are not too far apart!")

# Waiting for start of race
print("To start race press 'Space' key.")

while True:
        keyp = readkey()
        if keyp == ' ':
            print("Go")
            break

# Drive forward at full speed
sonarRFC.forward(speed)

print("Their off! Press Control^C to finish")

# Try to avoid the walls!
try:
    while True:
        # Calculate the distance from edge of robot to walls
        leftDistance = viewLeft.Measurement()
        print("Average measured left distance: " + str(int(leftDistance)) + " cm")
        rightDistance = viewRight.Measurement()
        print("Average measured right distance: " + str(int(rightDistance)) + " cm")
        angleFromWall = wallAngle(leftDistance, rightDistance)
        print("Angle from the wall: " + str(int(angleFromWall)) + " deg")
        spaceFromLeftWall = distanceFromWall(leftDistance, angleFromWall)
        print("Actual distance from the left wall: " + str(int(spaceFromLeftWall)) + " cm")

        # Decide which way to steer
        if (leftDistance < firstBufferWidth):
            sonarRFC.turnForward(speed, (speed - arcSpeed))
            print("Steering right")
        elif (rightDistance < firstBufferWidth):
            sonarRFC.turnForward((speed - arcSpeed), speed)
            print("Steering left")
        elif ((rightDistance < secondBufferWidth) and (leftDistance < secondBufferWidth)):
            sonarRFC.stop()
            print("The walls are caving in!")
        elif ((rightDistance > wallWidth) and (leftDistance > wallWidth)):
            sonarRFC.stop()
            print("I'm in the clear!")
        else:
            sonarRFC.forward(speed)
            print("Storming forward!")
        
        # Loop delay
        time.sleep(loopTime)
        
except KeyboardInterrupt:
    print("Stopping the race")

# Sets all motors off and sets GPIO to standard values
finally:
    sonarRFC.cleanup()
