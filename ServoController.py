#!/usr/bin/python

"""
Python Module to externalise all Initio/RoboHAT specific hardware
Created by Gareth Davies, Feb 2016
Copyright 4tronix

This code is in the public domain and may be freely copied and used
No warranty is provided or implied
Servo Functions
Pirocon/Microcon/RoboHAT use ServoD to control servos
"""

import logging
import os
import time

module_logger = logging.getLogger("__main__.ServoController")


class ServoController:

    """
    """
    # Define pins for Pan/Tilt
    PAN_SERVO = 0
    TILT_SERVO = 1

    def __init__(self):
        """
        """
        self.ServosActive = False
        module_logger.info("Setting up ServoController Module")

    def startServos(self):
        """
        """
        if self.ServosActive is False:
            SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
            SERVOD_CMD = '/servod --pcm --idle-timeout=20000 --p1pins="18,22"'
            initString = "sudo " + SCRIPTPATH + SERVOD_CMD + ' > /dev/null'
            os.system(initString)
            self.ServosActive = True

    def stopServos(self):
        """
        """
        os.system("sudo pkill -f servod")
        self.ServosActive = False

    def setServo(self, servo, degrees):
        """
        """
        if self.ServosActive is False:
            self.startServos()
        self.pinServod(servo, degrees)

    def setPanServer(self, degrees):
        """
        """
        self.setServo(self.PAN_SERVO, degrees)

    def setTiltServer(self, degrees):
        """
        """
        self.setServo(self.TILT_SERVO, degrees)

    def pinServod(self, pin, degrees):
        """
        """
        pinString = "echo " + str(pin) + "=" + str(
            50 + ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
        os.system(pinString)


if __name__ == "__main__":
    try:
        servo = ServoController()

        servo.setPanServer(0)
        servo.startServos()
        servo.setPanServer(0)
        time.sleep(1)
        servo.setPanServer(90)
        time.sleep(1)
        servo.setPanServer(-90)
        time.sleep(1)
        servo.setPanServer(0)
        time.sleep(1)

        servo.setTiltServer(0)
        time.sleep(1)
        servo.setTiltServer(90)
        time.sleep(1)
        servo.setTiltServer(-90)
        time.sleep(1)
        servo.setTiltServer(0)
    except KeyboardInterrupt:
        pass
    finally:
        servo.stopServos()
