#!/usr/bin/python
#
# Python Module for Robohat/Initio sensors not in 4tronix code.
# Additional functions for new sensors such as additional ultrasonic
# Library for straight line speed test
#
#======================================================================

#======================================================================
# General Functions
#
# initMotors(): Initialises GPIO pins
# cleanup(): Sets all motors off and sets GPIO to standard values
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
# UltraSonic Classes and Functions
#
# class rightUltraSensor(): Right hand side ultrasonic sensor
#       .Measurement() : Returns the distance in cm to the nearest reflecting object
# class leftUltraSensor(): Left hand side ultrasonic sensor
#       .Measurement() : Returns the distance in cm to the nearest reflecting object
# class frontUltraSensor(): Front ultrasonic sensor
#       .Measurement() : Returns the distance in cm to the nearest reflecting object
#======================================================================


# Import all necessary libraries
# Not importing robohat, but copying needed functions
from __future__ import division # Used for floating point division in Python 2.7
import RPi.GPIO as GPIO
import time
import Queue

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
# New robohat sensor pins - Physical/J5 Pin Numbers
#
#======================================================================

# UltraSonic physical pin numbers
sonarInpRight = 16 # Connected to Echo
sonarOutRight = 31 # Connected to Trig

sonarInpLeft = 15 # Connected to Echo
sonarOutLeft = 12 # Connected to Trig

# Running average settings
QSIZE = 8
QINITIAL = 30.0 # Based on wall to wall distance of ~60cm


#======================================================================
# General Functions

# initMotors(). Initialises GPIO pins
def initMotors():

    # Setup global variables needed for motors
    global p, q, a, b

    # Use physical pin numbering
    GPIO.setmode(GPIO.BOARD)

    # Disable warnings
    GPIO.setwarnings(False)

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


# cleanup(). Sets all motors off and sets GPIO to standard values
def cleanup():
    stop()
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
# UltraSonic Functions

# Right hand side ultrasonic sensor
class rightUltraSensor():

    def __init__(self):
        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)
            
        # Initialise GPIO pins
        GPIO.setup(sonarOutRight, GPIO.OUT)
        GPIO.setup(sonarInpRight,GPIO.IN)

        # Initilise Queue
        self.q = Queue.Queue()
        
        for i in xrange(0,QSIZE):
            self.q.put(QINITIAL)

    def Measurement(self):     # Returns the distance in cm to the nearest reflecting object

        # Send 10us pulse to trigger
        GPIO.output(sonarOutRight, True)
        time.sleep(0.00001)
        GPIO.output(sonarOutRight, False)
        start = time.time()
        count=time.time()

        # Measure echo
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
        # print("Right distance: " + str(distance))
        
        # Add latest distance to queue
        if (self.q.qsize() > QSIZE) :
            self.q.get()
        self.q.put(distance)
     
        # Calculate running average
        total = 0
        for val in self.q.queue:
            total = total + val
     
        average = total / self.q.qsize()
        # print("Right average: " + str(average))
        
        return average

# Left hand side ultrasonic sensor
class leftUltraSensor():

    def __init__(self):
        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)
            
        # Initialise GPIO pins
        GPIO.setup(sonarOutLeft, GPIO.OUT)
        GPIO.setup(sonarInpLeft,GPIO.IN)

        # Initilise Queue
        self.q = Queue.Queue()
        
        for i in xrange(0,QSIZE):
            self.q.put(QINITIAL)

    def Measurement(self):    # Returns the distance in cm to the nearest reflecting object

        # Send 10us pulse to trigger
        GPIO.output(sonarOutLeft, True)
        time.sleep(0.00001)
        GPIO.output(sonarOutLeft, False)
        start = time.time()
        count=time.time()

        # Measure echo
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
        # print("Left distance: " + str(distance))
        
        # Add latest distance to queue
        if (self.q.qsize() > QSIZE) :
            self.q.get()
        self.q.put(distance)
     
        # Calculate running average
        total = 0
        for val in self.q.queue:
            total = total + val
     
        average = total / self.q.qsize()
        # print("Left average: " + str(average))
        
        return average

# Front ultrasonic sensor
class frontUltraSensor():

    def __init__(self):
        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)

        # Initilise Queue
        self.q = Queue.Queue()
        
        for i in xrange(0,QSIZE):
            self.q.put(QINITIAL)

    def Measurement(self):    # Returns the distance in cm to the nearest reflecting object

        # Initialise GPIO pins
        GPIO.setup(sonar, GPIO.OUT)
        
        # Send 10us pulse to trigger
        GPIO.output(sonar, True)
        time.sleep(0.00001)
        GPIO.output(sonar, False)
        start = time.time()
        count=time.time()

        # Measure echo
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
        # multiplied by the speed of sound 34000(cm/s) divided by 2
        distance = elapsed * 17000
        # print("Left distance: " + str(distance))
        
        # Add latest distance to queue
        if (self.q.qsize() > QSIZE) :
            self.q.get()
        self.q.put(distance)
     
        # Calculate running average
        total = 0
        for val in self.q.queue:
            total = total + val
     
        average = total / self.q.qsize()
        # print("Left average: " + str(average))
        
        return average

# End of UltraSonic Functions    
#======================================================================


