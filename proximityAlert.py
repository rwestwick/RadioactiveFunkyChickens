#!/usr/bin/python
# proximityAlert.py

# This code is for use in the Proximity Alert PiWars 2015 challenge
# http://piwars.org/2015-competition/challenges/proximity-alert/

# Import required libraries
import robohat
import sys, time

# Define servo movement function
def doServos():
    robohat.setServo(pan, pVal)
    robohat.setServo(tilt, tVal)

# Set initial variables
speed = 40
minRange = 5
pan = 0
tilt = 1
tVal = 0 # 0 degrees is centre
pVal = 0 # 0 degrees is centre

# Initialise robohat controller
robohat.init()

# Set servos to point sonar to point straight forward
doServos() # Look straight ahead

# Measure current distance
dist = robohat.getDistance()
print "Distance to wall, ", dist
time.sleep(1)

while dist >= minRange:
    # robohat.forward(speed)
    print "Distance to wall, ", dist
    time.sleep(0.5)
    dist = robohat.getDistance()

print "I have decided to stop!!"

robohat.cleanup()
