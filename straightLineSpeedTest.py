#!/usr/bin/python
# straightLineSpeedTest.py

# This code is for use in the Straight Line Speed Test - PiWars 2017 challenge
# http://piwars.org/

# Import required libraries
import robohat
import sys, time

# Define servo movement function
def doServos():
    robohat.setServo(pan, pVal)
    robohat.setServo(tilt, tVal)

def lookLeftAndRight():
    pVal = -10
    robohat.setServo(tilt, -30)
    time.sleep(1)
    leftDistance = robohat.getDistance()
    print "Left distance, ", leftDistance
    robohat.setServo(tilt, 30)
    time.sleep(1)
    rightDistance = robohat.getDistance()
    print "Right distance, ", rightDistance
    robohat.setServo(tilt, 0)

# Set initial variables
speed = 40
minRange = 20
pan = 0
tilt = 1
tVal = 0 # 0 degrees is horizontal centre
pVal = 0 # 0 degrees is vertical centre

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
    pass

finally: # cleanup is run even if ^c is typed
    robohat.cleanup()
