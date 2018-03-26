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
import ConfigParser # pylint: disable=E0401
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
        MODULE_LOGGER.info("Setting up ServoController Module")
        self.servos_active = False
        self.pan_offset = 0
        self.pan_min = -90
        self.pan_max = 90
        self.tilt_offset = 0
        self.tilt_min = -75
        self.tilt_max = 90
        self.fire_offset = 0
        self.fire_min = -90
        self.fire_max = 90
        
        if not self.read_config_file():
            self.create_config_file()

        self.set_pan_servo(0)
        self.set_tilt_servo(0)
        self.set_fire_servo(0)

    def create_config_file(self):
        """
        Creates a config file with default (ie current) options
        """
        MODULE_LOGGER.info("Creating a config file")
        config = ConfigParser.ConfigParser()

        # lets create that config file for next time...
        config_file = open("config_servo.ini", 'w')

        # add the settings to the structure of the file, and lets write it out...
        config.add_section('Settings')
        config.set('Settings', 'PanOffset', str(self.pan_offset))
        config.set('Settings', 'PanMin', str(self.pan_min))
        config.set('Settings', 'PanMax', str(self.pan_max))
        config.set('Settings', 'TiltOffset', str(self.tilt_offset))
        config.set('Settings', 'TiltMin', str(self.tilt_min))
        config.set('Settings', 'TiltMax', str(self.tilt_max))
        config.set('Settings', 'FireOffset', str(self.fire_offset))
        config.set('Settings', 'FireMin', str(self.fire_min))
        config.set('Settings', 'FireMax', str(self.fire_max))
        config.write(config_file)
        config_file.close()

    def read_config_file(self):
        """
        Read the current config file
        """
        found_config = True
        
        try:
            config_file = open("config_servo.ini", 'r')            
            config = ConfigParser.ConfigParser()
            config.readfp(config_file)           
            self.pan_offset = config.getint('Settings', 'PanOffset')
            self.pan_min = config.getint('Settings', 'PanMin')
            self.pan_max = config.getint('Settings', 'PanMax')
            self.tilt_offset = config.getint('Settings', 'TiltOffset')
            self.tilt_min = config.getint('Settings', 'TiltMin')
            self.tilt_max = config.getint('Settings', 'TiltMax')
            self.fire_offset = config.getint('Settings', 'FireOffset')
            self.fire_min = config.getint('Settings', 'FireMin')
            self.fire_max = config.getint('Settings', 'FireMax')            
            config_file.close()
            
        except IOError:
            found_config = False
            MODULE_LOGGER.warn("Existing config file not found")
            
        return found_config
        
    def start_servos(self):
        """
        Starts the servos using servod
        """
        if self.servos_active is False:
            MODULE_LOGGER.info("Starting servos")
            script_path = os.path.split(os.path.realpath(__file__))[0]
            servod_cmd = '/servod --idle-timeout=20000 --p1pins="' + \
                str(GPIOLayout.SERVO_PAN_PIN) + ',' + \
                str(GPIOLayout.SERVO_TILT_PIN) + ',' + \
                str(GPIOLayout.DUCK_SHOOT_FIRE_PIN) + \
                '"'  # With PWM hardware
            init_string = "sudo " + script_path + servod_cmd + ' > /dev/null &'
            MODULE_LOGGER.debug(init_string)
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
        if degrees < self.pan_min:
            MODULE_LOGGER.warn("Pan using min level of " + str(self.pan_min))
            degrees = self.pan_min
        elif degrees > self.pan_max:
            MODULE_LOGGER.warn("Pan sing above max level of " + str(self.pan_max))
            degrees = self.pan_max
        else
            MODULE_LOGGER.info("Pan: " + str(degrees))

        self.set_servo(self.PAN_SERVO_ID, degrees + self.pan_offset)

    def set_tilt_servo(self, degrees):
        """
        Sets the tilt servo to a position
        """
        if degrees < self.tilt_min:
            MODULE_LOGGER.warn("Tilt using min level of " + str(self.tilt_min))
            degrees = self.tilt_min
        elif degrees > self.tilt_max:
            MODULE_LOGGER.warn("Tilt using above max level of " + str(self.tilt_max))
            degrees = self.tilt_max
        else
            MODULE_LOGGER.info("Tilt: " + str(degrees))

        self.set_servo(self.TILT_SERVO_ID, degrees + self.tilt_offset)

    def set_nerf_trigger_servo(self, degrees):
        """
        Sets the Nerf trigger servo
        """
        if degrees < self.fire_min:
            MODULE_LOGGER.warn("Fire using min level of " + str(self.fire_min))
            degrees = self.fire_min
        elif degrees > self.fire_max:
            MODULE_LOGGER.warn("Fire using above max level of " + str(self.fire_max))
            degrees = self.fire_max
        else
            MODULE_LOGGER.info("Fire: " + str(degrees))

        self.set_servo(self.NERF_TRIGGER_ID, degrees + self.fire_offset)

    def pin_servod(self, pin, degrees):
        """
        Uses the servoblaster device to set the required angle
        eg  echo p1-18=120 > /dev/servoblaster
        """
        PIN_STRING = 'echo p1-'
        if pin == self.PAN_SERVO_ID:
            PIN_STRING = PIN_STRING + str(GPIOLayout.SERVO_PAN_PIN) + '='

        elif pin == self.TILT_SERVO_ID:
            PIN_STRING = PIN_STRING + str(GPIOLayout.SERVO_TILT_PIN) + '='

        else:
            PIN_STRING = PIN_STRING + str(GPIOLayout.DUCK_SHOOT_FIRE_PIN) + '='

        PIN_STRING = PIN_STRING + str(50 + (
            (90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
        MODULE_LOGGER.debug(PIN_STRING)
        os.system(PIN_STRING)
