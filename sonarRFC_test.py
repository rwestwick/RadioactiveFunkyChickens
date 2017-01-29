#!/usr/bin/python
#
# Python test code for sonarRFC.py module basic testing.
#
#======================================================================

# Import all necessary libraries
import sonarRFC
import time

# Create necessary objects
viewLeft = sonarRFC.leftUltraSensor()
viewRight = sonarRFC.rightUltraSensor()
viewFront = sonarRFC.frontUltraSensor()
time.sleep(1)

# Take repeated measurements and print results
for i in range(20):
    leftDistance = viewLeft.Measurement()
    rightDistance = viewRight.Measurement()
    frontDistance = viewFront.Measurement()
    
    print("Average left distance: " + str(leftDistance) + " cm")
    time.sleep(1)
    print("Average right distance: " + str(rightDistance) + " cm")
    time.sleep(1)
    print("Average front distance: " + str(frontDistance) + " cm")
    time.sleep(1)



