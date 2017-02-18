#!/usr/bin/python

"""
Python Module to externalise all Initio/RoboHAT specific hardware
Created by Gareth Davies, Feb 2016
Copyright 4tronix

This code is in the public domain and may be freely copied and used
No warranty is provided or implied
servo_controller Functions
Pirocon/Microcon/RoboHAT use ServoD to control servos
"""

import logging
import os
import time

MODULE_LOGGER = logging.getLogger("__main__.ServoController")


class ServoController(object):

    """
    Class to control the two (pan/tilt) servos on the initio robot
    """
    # Define pins for Pan/Tilt
    PAN_SERVO = 0
    TILT_SERVO = 1

    def __init__(self):
        """
        Initialises the required properties of the servo controller
        """
        self.servos_active = False
        MODULE_LOGGER.info("Setting up ServoController Module")

    def start_servos(self):
        """
        Starts the servos using servod
        """
        if self.servos_active is False:
            script_path = os.path.split(os.path.realpath(__file__))[0]
            servod_cmd = '/servod --pcm --idle-timeout=20000 --p1pins="18,22"'
            init_string = "sudo " + script_path + servod_cmd + ' > /dev/null'
            os.system(init_string)
            self.servos_active = True

    def stop_servos(self):
        """
        Stops the servos by killing the servod binary
        """
        os.system("sudo pkill -f servod")
        self.servos_active = False

    def set_servo(self, servo, degrees):
        """
        Sets a specific servo to a position, starting the servo if required
        """
        if self.servos_active is False:
            self.start_servos()
        self.pin_servod(servo, degrees)

    def set_pan_servo(self, degrees):
        """
        Sets the pan servo to a position
        """
        self.set_servo(self.PAN_SERVO, degrees)

    def set_tilt_servo(self, degrees):
        """
        Sets the tilt servo to a position
        """
        self.set_servo(self.TILT_SERVO, degrees)

    @staticmethod
    def pin_servod(pin, degrees):
        """
        Uses the servoblaster device to set the required angle
        """
        pin_string = "echo " + str(pin) + "=" + str(
            50 + ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
        os.system(pin_string)


if __name__ == "__main__":
    try:
        SERVO_CONTROLLER = ServoController()

        SERVO_CONTROLLER.set_pan_servo(0)
        SERVO_CONTROLLER.start_servos()
        SERVO_CONTROLLER.set_pan_servo(0)
        time.sleep(1)
        SERVO_CONTROLLER.set_pan_servo(90)
        time.sleep(1)
        SERVO_CONTROLLER.set_pan_servo(-90)
        time.sleep(1)
        SERVO_CONTROLLER.set_pan_servo(0)
        time.sleep(1)

        SERVO_CONTROLLER.set_tilt_servo(0)
        time.sleep(1)
        SERVO_CONTROLLER.set_tilt_servo(90)
        time.sleep(1)
        SERVO_CONTROLLER.set_tilt_servo(-90)
        time.sleep(1)
        SERVO_CONTROLLER.set_tilt_servo(0)
    except KeyboardInterrupt:
        pass
    finally:
        SERVO_CONTROLLER.stop_servos()
