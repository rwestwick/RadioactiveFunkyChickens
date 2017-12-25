#!/usr/bin/python
# straightLineSpeedTest.py

# This code is for use in the Straight Line Speed Test - PiWars 2017 challenge
# http://piwars.org/

# Import required libraries
import robohat
import time

#
# Define local functions #
#

# Define servo movement function

def doServos():
    robohat.setServo(pan, pVal)
    robohat.setServo(tilt, tVal)
    return

# Measure left and right wall distances

def lookLeftAndRight(wallList):  # wallList passed by reference

    # Measure left wall
    robohat.setServo(tilt, -90)
    time.sleep(0.5)
    leftDistance = robohat.getDistance()

    # Measure right wall
    robohat.setServo(tilt, 90)
    time.sleep(0.5)
    rightDistance = robohat.getDistance()

    # recenter servo
    robohat.setServo(tilt, 0)

    # Update wall measurements
    wallList[0] = leftDistance
    wallList[1] = rightDistance

    return

#
# Main program           #
#

# Set initial variables
speed = 40            # Initial forward speed
arcSpeed = 20         # Difference between left and right for correction
minRange = 20         # Minimum range in front of robot
pan = 0               # Value for setServo first argument for pan
tilt = 1              # Value for setServo first argument for pan
tVal = 0              # 0 degrees is horizontal centre
pVal = 0              # 0 degrees is vertical centre
wallList = [0, 0]     # Left and right wall distance List
# Width of the wall in cm (actual width on web-page = 522 mm)
wallWidth = 52

# Initialise robohat controller
robohat.init()

# Set servos to point sonar to initial chosen direction
doServos()

# Measure current distance to front wall
frontDist = robohat.getDistance()
print "Distance to front wall, ", int(frontDist)

# Measure current distance to side walls
lookLeftAndRight(wallList)
print "Left and right wall distances: ", wallList

# Calculate trough dimensions
troughWidth = wallList[0] + wallList[1]
print "Trough width: ", int(troughWidth)

if ((troughWidth > (wallWidth + minRange))
        or (troughWidth < (wallWidth - minRange))):
    print "Trough walls not detected properly."

# Control loop
try:
    while True:
        if (wallList[0] < minRange):
            print "Distance to left wall, ", int(wallList[0])
            print "Turning right."
            # robohat.turnForward((speed + arcSpeed), speed)
            time.sleep(0.5)
        elif (wallList[1] < minRange):
            print "Distance to right wall, ", int(wallList[1])
            print "Turning left."
            # robohat.turnForward()robohat.turnForward(speed, (speed +
            # arcSpeed))
            time.sleep(0.5)

        # Move forward and remeasure
        # robohat.forward(speed)
        time.sleep(1)
        lookLeftAndRight(wallList)
        print "Left and right wall distances: ", wallList

except KeyboardInterrupt:
    robohat.stop()
    doServos()
    time.sleep(1)
    print "Exiting"

finally:  # cleanup is run even if ^c is typed
    robohat.cleanup()
