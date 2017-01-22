#!/usr/bin/python
#
# Basic Test for sonar via general GPIO.
#
#======================================================================

# Import all necessary libraries
import funkyChicken
import time

funkyChicken.init()

for i in range(10):
    testDistance = funkyChicken.getRightDistance()
    testDistance = int(testDistance)
    print "Right distance: ", testDistance, "cm"
    time.sleep(1)

for i in range(10):
    testDistance = funkyChicken.getLeftDistance()
    testDistance = int(testDistance)
    print "Left distance: ", testDistance, "cm"
    time.sleep(1)

funkyChicken.cleanup()
