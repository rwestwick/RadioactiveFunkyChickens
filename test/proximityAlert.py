#!/usr/bin/python
# proximityAlert.py

# This code is for use in the Proximity Alert PiWars 2015 challenge
# http://piwars.org/2015-competition/challenges/proximity-alert/

# Import required libraries
import robohat
import time

# Define servo movement function

def doServos():
    robohat.setServo(pan, pVal)
    robohat.setServo(tilt, tVal)

# Set initial variables
speed = 40
minRange = 10
pan = 0
tilt = 1
tVal = 0  # 0 degrees is horizontal centre
pVal = 0  # 0 degrees is vertical centre

# Initialise robohat controller
robohat.init()

# Set servos to point sonar to initial chosen direction
doServos()

# Measure current distance
dist = robohat.getDistance()
print "Distance to wall, ", int(dist)
time.sleep(1)

try:
    while True:
        if dist >= minRange:
            robohat.forward(speed)
            print "Stepping forward"
            print "Distance to wall, ", int(dist)
            time.sleep(0.5)
            dist = robohat.getDistance()
        else:
            robohat.stop()
            print "hello handsome"
            dist = robohat.getDistance()
            print "Distance to wall, ", int(dist)
            time.sleep(0.5)

except KeyboardInterrupt:
    print "Exiting"

finally:  # cleanup is run even if ^c is typed
    robohat.cleanup()
