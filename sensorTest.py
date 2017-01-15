#!/usr/bin/python
#
# Basic Test for sonar via general GPIO.
#
#======================================================================

# Import all necessary libraries
import RPi.GPIO as GPIO
import time

# getDistance(): Returns the distance in cm to the nearest reflecting object
def getDistance():
    GPIO.setup(sonarOut, GPIO.OUT)
    
    # Send 10us pulse to trigger
    GPIO.output(sonarOut, True)
    time.sleep(0.00001)
    GPIO.output(sonarOut, False)
    start = time.time()
    count=time.time()
    
    GPIO.setup(sonarInp,GPIO.IN)
    while GPIO.input(sonarInp)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonarInp)==1 and time.time()-count<0.1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    return distance

#use physical pin numbering
GPIO.setmode(GPIO.BOARD)

# UltraSonic physical pin numbers
sonarInp = 16 # Connected to Echo
sonarOut = 31 # Connected to Trig

for i in range(10):
    testDistance = getDistance()
    testDistance = int(testDistance)
    print "Distance: ", testDistance, "cm"
    time.sleep(1)

GPIO.cleanup()
