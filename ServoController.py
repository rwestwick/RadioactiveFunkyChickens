#!/usr/bin/python
"""
Python Module to externalise all Initio/RoboHAT specific hardware
Created by Gareth Davies, Feb 2016
Copyright 4tronix

This code is in the public domain and may be freely copied and used
No warranty is provided or implied
servo_controller Functions
Pirocon/Microcon/RoboHAT use ServoD to control servos
Old version?
https://github.com/richardghirst/PiBits/tree/master/ServoBlaster
servod -help
"""

import logging
import os
import GPIOLayout

MODULE_LOGGER = logging.getLogger("__main__.ServoController")


class ServoController(object):
    """
    Class to control the two (pan/tilt) servos on the initio robot
    """
    # Define pins for Pan/Tilt
    PAN_SERVO_ID = 0
    TILT_SERVO_ID = 1
    NERF_TRIGGER_ID = 2

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
            servod_cmd = '/servod --idle-timeout=20000 --p1pins="' + \
                str(GPIOLayout.SERVO_HORIZONTAL_PIN) + ',' + \
                str(GPIOLayout.SERVO_VERTICAL_PIN) + ',' + \
                str(GPIOLayout.DUCK_SHOOT_FIRE_GPIO) + \
                '"'  # With PWM hardware
            init_string = "sudo " + script_path + servod_cmd + ' > /dev/null &'
            os.system(init_string)
            self.servos_active = True

    def stop_servos(self):
        """
        Stops the servos by killing the servod binary
        """
        os.system("sudo pkill -f servod")
        self.servos_active = False
        MODULE_LOGGER.info("ServoController Module Stopped")

    def set_servo(self, servo_id, degrees):
        """
        Sets a specific servo to a position, starting the servo if required
        """
        if self.servos_active is False:
            self.start_servos()
        self.pin_servod(servo_id, degrees)

    def set_pan_servo(self, degrees):
        """
        Sets the pan servo to a position
        """
        self.set_servo(self.PAN_SERVO_ID, degrees)

    def set_tilt_servo(self, degrees):
        """
        Sets the tilt servo to a position
        """
        self.set_servo(self.TILT_SERVO_ID, degrees)
        
    def set_nerf_trigger_servo(self, degrees):
        """
        Sets the Nerf trigger 
        """
        self.set_servo(self.NERF_TRIGGER_ID, degrees)

    @staticmethod
    def pin_servod(pin, degrees):
        """
        Uses the servoblaster device to set the required angle
        eg  echo p1-18=120 > /dev/servoblaster
        """
        PIN_STRING = 'echo p1-'
        if pin == 0:
            PIN_STRING = PIN_STRING + str(GPIOLayout.SERVO_HORIZONTAL_PIN) + '='
        elif pin == 1:
            PIN_STRING = PIN_STRING + str(GPIOLayout.SERVO_VERTICAL_PIN) + '='
        else:
            PIN_STRING = PIN_STRING + str(GPIOLayout.DUCK_SHOOT_FIRE_GPIO) + '='

        PIN_STRING = PIN_STRING + str(50 + (
            (90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
        os.system(PIN_STRING)
