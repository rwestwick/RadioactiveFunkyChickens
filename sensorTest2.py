#!/usr/bin/python
#
# Basic Test for sonar via general GPIO.
# https://www.modmypi.com/blog/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi
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
    
    GPIO.setup(sonarInp,GPIO.IN)
    while GPIO.input(sonarInp)==0:
        pulse_start = time.time()
    
    while GPIO.input(sonarInp)==1:
        pulse_end = time.time()

    # Calculate pulse length
    pulse_duration = pulse_end - pulse_start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34300(cm/s) divided by 2
    distance = pulse_duration * 17150
    return distance

#use physical pin numbering
GPIO.setmode(GPIO.BOARD)

# UltraSonic physical pin numbers
sonarInp = 16 # Connected to Echo
sonarOut = 31 # Connected to Trig

for i in range(10):
    testDistance = getDistance()
    testDistance = round(testDistance, 2)
    print "Distance: ", testDistance, "cm"
    time.sleep(1)

GPIO.cleanup()
