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
# Motor Functions
#
# stop(): Stops both motors
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# turnreverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
# irRight(): Returns state of Right IR Obstacle sensor
# irAll(): Returns true if either of the Obstacle sensors are triggered
# irLeftLine(): Returns state of Left IR Line sensor
# irRightLine(): Returns state of Right IR Line sensor
#======================================================================


#======================================================================
# Servo Functions
# 
# startServos(). Initialises the servo background process
# stop Servos(). terminates the servo background process
# setServo(Servo, Degrees). Sets the servo to position in degrees -90 to +90
#======================================================================


#======================================================================
# Wheel Sensor Functions
# !! These functions are now commented out due to conflicts in pin numbers with additional ultrasonic sensors
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
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
#======================================================================


#======================================================================
# Mathematical Functions
#
# runningAvg(nextValue): Maintains a running average of the last lengthOfList nextValues
#======================================================================


# Import all necessary libraries
# Not importing robohat, but copying needed functions and init
from __future__ import division # Used for floating point division in Python 2.7
import RPi.GPIO as GPIO
import sys
import threading
import time
import os

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

ServosActive = False

#======================================================================
# New robohat sensor pins - Physical/J5 Pin Numbers
#
#======================================================================

# UltraSonic physical pin numbers
sonarInpRight = 16 # Connected to Echo
sonarOutRight = 31 # Connected to Trig

sonarInpLeft = 15 # Connected to Echo
sonarOutLeft = 12 # Connected to Trig

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

    # Setup global variables needed for motors
    global p, q, a, b

    #use physical pin numbering
    GPIO.setmode(GPIO.BOARD)

    # Disable warnings
    GPIO.setwarnings(False)

    #set up digital line detectors as inputs
    GPIO.setup(lineRight, GPIO.IN) # Right line sensor
    GPIO.setup(lineLeft, GPIO.IN) # Left line sensor

    #Set up IR obstacle sensors as inputs
    GPIO.setup(irFL, GPIO.IN) # Left obstacle sensor
    GPIO.setup(irFR, GPIO.IN) # Right obstacle sensor

    #use pwm on inputs so motors don't go too fast
    GPIO.setup(L1, GPIO.OUT)
    p = GPIO.PWM(L1, 20)
    p.start(0)

    GPIO.setup(L2, GPIO.OUT)
    q = GPIO.PWM(L2, 20)
    q.start(0)

    GPIO.setup(R1, GPIO.OUT)
    a = GPIO.PWM(R1, 20)
    a.start(0)

    GPIO.setup(R2, GPIO.OUT)
    b = GPIO.PWM(R2, 20)
    b.start(0)

    startServos()

    # Start threading for wheel counter
    # threadC = threading.Thread(target = wheelCount)
    # threadC.start()
    # running = True


# cleanup(). Sets all motors off and sets GPIO to standard values
def cleanup():
    global running
    running = False
    stop()
    stopServos()
    time.sleep(1)
    GPIO.cleanup()


# End of General Functions
#======================================================================

#======================================================================
# Motor Functions
#
# stop(): Stops both motors
def stop():
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(0)
    
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
def forward(speed):
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(speed + 5)
    a.ChangeFrequency(speed + 5)
    
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
def reverse(speed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    q.ChangeFrequency(speed + 5)
    b.ChangeFrequency(speed + 5)

# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinLeft(speed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(speed)
    a.ChangeDutyCycle(speed)
    b.ChangeDutyCycle(0)
    q.ChangeFrequency(speed + 5)
    a.ChangeFrequency(speed + 5)
    
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinRight(speed):
    p.ChangeDutyCycle(speed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(speed)
    p.ChangeFrequency(speed + 5)
    b.ChangeFrequency(speed + 5)
    
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnForward(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(leftSpeed)
    q.ChangeDutyCycle(0)
    a.ChangeDutyCycle(rightSpeed)
    b.ChangeDutyCycle(0)
    p.ChangeFrequency(leftSpeed + 5)
    a.ChangeFrequency(rightSpeed + 5)
    
# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnReverse(leftSpeed, rightSpeed):
    p.ChangeDutyCycle(0)
    q.ChangeDutyCycle(leftSpeed)
    a.ChangeDutyCycle(0)
    b.ChangeDutyCycle(rightSpeed)
    q.ChangeFrequency(leftSpeed + 5)
    b.ChangeFrequency(rightSpeed + 5)

# End of Motor Functions
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
def irLeft():
    if GPIO.input(irFL)==0:
        return True
    else:
        return False
    
# irRight(): Returns state of Right IR Obstacle sensor
def irRight():
    if GPIO.input(irFR)==0:
        return True
    else:
        return False
    
# irAll(): Returns true if any of the Obstacle sensors are triggered
def irAll():
    if GPIO.input(irFL)==0 or GPIO.input(irFR)==0:
        return True
    else:
        return False
    
# irLeftLine(): Returns state of Left IR Line sensor
def irLeftLine():
    if GPIO.input(lineLeft)==0:
        return True
    else:
        return False
    
# irRightLine(): Returns state of Right IR Line sensor
def irRightLine():
    if GPIO.input(lineRight)==0:
        return True
    else:
        return False
    
# End of IR Sensor Functions
#======================================================================

#======================================================================
# Servo Functions
# Pirocon/Microcon/RoboHAT use ServoD to control servos

def setServo(Servo, Degrees):
    global ServosActive
    #print "ServosActive:", ServosActive
    #print "Setting servo"
    if ServosActive == False:
        startServos()
    pinServod (Servo, Degrees) # for now, simply pass on the input values

def stopServos():
    #print "Stopping servo"
    stopServod()
    
def startServos():
    #print "Starting servod as CPU =", CPU
    startServod()
    
def startServod():
    global ServosActive
    #print "Starting servod. ServosActive:", ServosActive
    SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
    #os.system("sudo pkill -f servod")
    initString = "sudo " + SCRIPTPATH +'/servod --pcm --idle-timeout=20000 --p1pins="18,22" > /dev/null'
    os.system(initString)
    #print initString
    ServosActive = True

def pinServod(pin, degrees):
    #print pin, degrees
    pinString = "echo " + str(pin) + "=" + str(50+ ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
    #print pinString
    os.system(pinString)
    
def stopServod():
    global ServosActive
    os.system("sudo pkill -f servod")
    ServosActive = False

# End of Servo Functions
#======================================================================

#======================================================================
# Wheel Sensor Functions

# p, q, a, b are global variables for software PWM,
# e.g. p = GPIO.PWM(L1, 20) as used in pi2go.py and robohat.py
##def stopL():
##    p.ChangeDutyCycle(0)
##    q.ChangeDutyCycle(0)
##
##def stopR():
##    a.ChangeDutyCycle(0)
##    b.ChangeDutyCycle(0)
##
##def wheelCount():
##    global running, countL, countR
##    lastValidL = 2
##    lastValidR = 2
##    lastL = GPIO.input(lineLeft)
##    lastR = GPIO.input(lineRight)
##    while running:
##        time.sleep(0.002)
##        val = GPIO.input(lineLeft)
##        if val == lastL and val != lastValidL:
##            countL += 1
##            lastValidL = val
##        lastL = val
##        val = GPIO.input(lineRight)
##        if val == lastR and val != lastValidR:
##            countR += 1
##            lastValidR = val
##        lastR = val
##
##
### stepForward(speed, steps): Moves forward specified number of counts, then stops
##def stepForward(speed, counts):
##    global countL, countR
##    countL = 0
##    countR = 0
##    runL = True
##    runR = True
##    turnForward(speed, speed)
##    while runL or runR:
##        time.sleep(0.002)
##        if countL >= counts:
##            stopL()
##            runL = False
##        if countR >= counts:
##            stopR()
##            runR = False
##            
### stepReverse(speed, steps): Moves backward specified number of counts, then stops
##def stepReverse(speed, counts):
##    global countL, countR
##    countL = 0
##    countR = 0
##    runL = True
##    runR = True
##    turnReverse(speed, speed)
##    while runL or runR:
##        time.sleep(0.002)
##        if countL >= counts:
##            stopL()
##            runL = False
##        if countR >= counts:
##            stopR()
##            runR = False
##            
### stepSpinL(speed, steps): Spins left specified number of counts, then stops
##def stepSpinL(speed, counts):
##    global countL, countR
##    countL = 0
##    countR = 0
##    spinLeft(speed)
##    while countL<counts or countR<counts:
##        time.sleep(0.002)
##        if countL >= counts:
##            stopL()
##        if countR >= counts:
##            stopR()
##            
### stepSpinR(speed, steps): Spins right specified number of counts, then stops
##def stepSpinR(speed, counts):
##    global countL, countR
##    countL = 0
##    countR = 0
##    spinRight(speed)
##    while countL<counts or countR<counts:
##        time.sleep(0.002)
##        if countL >= counts:
##            stopL()
##        if countR >= counts:
##            stopR()


# End of Wheel Sensor Functions
#======================================================================


#======================================================================
# UltraSonic Functions

# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
def getDistance():
    GPIO.setup(sonar, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonar, True)
    time.sleep(0.00001)
    GPIO.output(sonar, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonar,GPIO.IN)
    while GPIO.input(sonar)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonar)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s)
    distance = elapsed * 34000
    # That was the distance there and back so halve the value
    distance = distance / 2
    return distance

# getRightDistance(): Returns the distance in cm to the nearest reflecting object
def getRightDistance():
    GPIO.setup(sonarOutRight, GPIO.OUT)
    
    # Send 10us pulse to trigger
    GPIO.output(sonarOutRight, True)
    time.sleep(0.00001)
    GPIO.output(sonarOutRight, False)
    start = time.time()
    count=time.time()
    
    GPIO.setup(sonarInpRight,GPIO.IN)
    while GPIO.input(sonarInpRight)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonarInpRight)==1 and time.time()-count<0.1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop-start

    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound 34000(cm/s) divided by 2
    distance = elapsed * 17000
    return distance

# getLeftDistance(): Returns the distance in cm to the nearest reflecting object
def getLeftDistance():
    GPIO.setup(sonarOutLeft, GPIO.OUT)
    
    # Send 10us pulse to trigger
    GPIO.output(sonarOutLeft, True)
    time.sleep(0.00001)
    GPIO.output(sonarOutLeft, False)
    start = time.time()
    count=time.time()
    
    GPIO.setup(sonarInpLeft,GPIO.IN)
    while GPIO.input(sonarInpLeft)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonarInpLeft)==1 and time.time()-count<0.1:
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
