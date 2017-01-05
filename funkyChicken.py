#!/usr/bin/python
#
# Python Module for Robohat/Initio sensors not in 4tronix code.
# Additional functions for new sensors such as additional ultrasonic
#
#======================================================================


#======================================================================
# General Functions
#
# init(): Initialises GPIO pins, switches motors and LEDs Off, etc
# cleanup(): Sets all motors and LEDs off and sets GPIO to standard values
#======================================================================


#======================================================================
# Wheel Sensor Functions
#
# stepForward(speed, steps): moves the unit forwards specified number of steps, then stops
# stepReverse(speed, steps): Moves backward specified number of counts, then stops
# stepSpinL(speed, steps): Spins left specified number of counts, then stops
# stepSpinR(speed, steps): Spins right specified number of counts, then stops
#======================================================================


#======================================================================
# UltraSonic Functions
#
# getLeftDistance(): Returns the distance in cm to the nearest reflecting object on the left
# getRightDistance(): Returns the distance in cm to the nearest reflecting object on the right
#======================================================================


#======================================================================
# Mathematical Functions
#
# runningAvg(nextValue): Maintains a running average of the last lengthOfList nextValues
#======================================================================


# Import all necessary libraries
from __future__ import division # Used for floating point division in Python 2.7
import RPi.GPIO as GPIO
import sys
import threading
import time
import os
import robohat

# Global variables for wheel sensor counting
running = True
countL = 0
countR = 0

#======================================================================
# Existing robohat.py sensor pins - J5 Pin Numbers
#
#======================================================================

# Pins 35, 36 Left Motor
# Pins 32, 33 Right Motor
L1 = 36
L2 = 35
R1 = 33
R2 = 32

# Define obstacle sensors and line sensors
# These can be on any input pins, but this library assumes the following layout
# which matches the build instructions
irFL = 7
irFR = 11
# lineLeft = 29
# lineRight = 13

# Define Sonar Pin (Uses same pin for both Ping and Echo)
sonar = 38

#======================================================================
# New robohat sensor pins - J5 Pin Numbers
#
#======================================================================

# Define Sonar Pins
sonarLeftInp = 15
sonarLeftOut = 9
sonarRightInp = 12
sonarRightOut = 31

# Define Wheel Sensor Pins
# Orange or Brown to the selected pin (in fact, each wheel has 2 sensors
# that are slightly out of phase so you can get direction of travel from them,
# but you will then need 2 pins for each sensor)
# N.B. Need to change variable names to avoid confusion!
lineLeft = 29
lineRight = 13 

# Global variables for mathematical functions
lengthOfList = 5
listAvg = [0] * lengthOfList
cntr = 0


#======================================================================
# General Functions

# init(). Initialises GPIO pins
def init():

    threadC = threading.Thread(target = wheelCount)
    threadC.start()
    running = True


# cleanup(). Sets all motors and LEDs off and sets GPIO to standard values
def cleanup():
    global running
    running = False
    stop()
    time.sleep(1)
    GPIO.cleanup()


# End of General Functions
#======================================================================


#======================================================================
# Wheel Sensor Functions

# p, q, a, b are global variables for software PWM,
# e.g. p = GPIO.PWM(L1, 20) as used in pi2go.py and robohat.py
def stopL():
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)

def stopR():
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(0)

def wheelCount():
    global running, countL, countR
    lastValidL = 2
    lastValidR = 2
    lastL = GPIO.input(lineLeft)
    lastR = GPIO.input(lineRight)
    while running:
        time.sleep(0.002)
        val = GPIO.input(lineLeft)
        if val == lastL and val != lastValidL:
            countL += 1
            lastValidL = val
        lastL = val
        val = GPIO.input(lineRight)
        if val == lastR and val != lastValidR:
            countR += 1
            lastValidR = val
        lastR = val


# stepForward(speed, steps): Moves forward specified number of counts, then stops
def stepForward(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    runL = True
    runR = True
    turnForward(speed, speed)
    while runL or runR:
        time.sleep(0.002)
        if countL >= counts:
            stopL()
            runL = False
        if countR >= counts:
            stopR()
            runR = False
            
# stepReverse(speed, steps): Moves backward specified number of counts, then stops
def stepReverse(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    runL = True
    runR = True
    turnReverse(speed, speed)
    while runL or runR:
        time.sleep(0.002)
        if countL >= counts:
            stopL()
            runL = False
        if countR >= counts:
            stopR()
            runR = False
            
# stepSpinL(speed, steps): Spins left specified number of counts, then stops
def stepSpinL(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    spinLeft(speed)
    while countL<counts or countR<counts:
        time.sleep(0.002)
        if countL >= counts:
            stopL()
        if countR >= counts:
            stopR()
            
# stepSpinR(speed, steps): Spins right specified number of counts, then stops
def stepSpinR(speed, counts):
    global countL, countR
    countL = 0
    countR = 0
    spinRight(speed)
    while countL<counts or countR<counts:
        time.sleep(0.002)
        if countL >= counts:
            stopL()
        if countR >= counts:
            stopR()


# End of Motor Functions
#======================================================================


#======================================================================
# UltraSonic Functions
sonarRightInp = 10
sonarRightOut = 11

# getLeftDistance(): Returns the distance in cm to the nearest reflecting object on the left
def getLeftDistance():
    GPIO.setup(sonarLeftOut, GPIO.OUT)
    
    # Send 10us pulse to trigger
    GPIO.output(sonarLeftOut, True)
    time.sleep(0.00001)
    GPIO.output(sonarLeftOut, False)
    start = time.time()
    count=time.time()
    
    GPIO.setup(sonarLeftInp,GPIO.IN)
    while GPIO.input(sonarLeftInp)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonarLeftInp)==1 and time.time()-count<0.1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    return distance

# getRightDistance(): Returns the distance in cm to the nearest reflecting object on the right
def getRightDistance():
    GPIO.setup(sonarRightOut, GPIO.OUT)
    
    # Send 10us pulse to trigger
    GPIO.output(sonarRightOut, True)
    time.sleep(0.00001)
    GPIO.output(sonarRightOut, False)
    start = time.time()
    count=time.time()
    
    GPIO.setup(sonarRightOut,GPIO.IN)
    while GPIO.input(sonarRightOut)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonarRightOut)==1 and time.time()-count<0.1:
        stop = time.time()
        
    # Calculate pulse length
    elapsed = stop-start
    
    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    return distance

# End of UltraSonic Functions    
#======================================================================


#======================================================================
# Mathematical Functions

# runningAvg(nextValue): Maintains a running average of the last lengthOfList nextValues
def runningAvg(nextValue):

    # Make sure relevant variables are not local
    global lengthOfList
    global listAvg
    global cntr
    
    listAvg[cntr] = float(nextValue)
    Avg = float(sum(listAvg))/len(listAvg)
    cntr = (cntr + 1) % lengthOfList
    
    return Avg

# End of Mathematical Functions
#======================================================================
