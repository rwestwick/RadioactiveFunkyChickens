#!/usr/bin/python
#
# Python Module to externalise all Initio/RoboHAT specific hardware
#
# Created by Gareth Davies, Feb 2016
# Copyright 4tronix
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
#======================================================================

#======================================================================
# Servo Functions
# 
# startServos(). Initialises the servo background process
# stop Servos(). terminates the servo background process
# setServo(Servo, Degrees). Sets the servo to position in degrees -90 to +90
#======================================================================


#======================================================================
# Servo Functions
# Pirocon/Microcon/RoboHAT use ServoD to control servos

import logging
import os
import time

# Define pins for Pan/Tilt
PAN_SERVO = 0
TILT_SERVO = 1

module_logger = logging.getLogger("__main__.ServoController")


class ServoController:
    """
    """
    
    def __init__(self):
        """
        """
        self.ServosActive = False
        module_logger.info("Setting up ServoController Module")

    def startServos():
        """
        """
        startServod()
        
    def startServod():
        """
        """
        SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
        initString = "sudo " + SCRIPTPATH +'/servod --pcm --idle-timeout=20000 --p1pins="18,22" > /dev/null'
        os.system(initString)
        self.ServosActive = True

    def stopServos():
        """
        """
        stopServod()

    def stopServod():
        """
        """
        os.system("sudo pkill -f servod")
        self.ServosActive = False

    def setServo(Servo, Degrees):
        """
        """
        if self.ServosActive == False:
            startServos()
        pinServod(Servo, Degrees) # for now, simply pass on the input values

    def pinServod(pin, degrees):
        """
        """
        pinString = "echo " + str(pin) + "=" + str(50+ ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
        os.system(pinString)

    
if __name__ == "__main__":
    try:
        ServoController servo
        servo.startServos()
        servo.setServo(PAN_SERVO, 0)
        time.sleep(1)
        servo.setServo(PAN_SERVO, 90)
        time.sleep(1)
        servo.setServo(PAN_SERVO, -90)
        time.sleep(1)
        servo.setServo(PAN_SERVO, 0)
        time.sleep(1)

        servo.setServo(TILT_SERVO, 0)
        time.sleep(1)
        servo.setServo(TILT_SERVO, 90)
        time.sleep(1)
        servo.setServo(TILT_SERVO, -90)
        time.sleep(1)
        servo.setServo(TILT_SERVO, 0)
    except KeyboardInterrupt:
        pass
    finally:
        servo.stopServos()
