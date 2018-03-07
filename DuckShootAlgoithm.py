#!/usr/bin/python
"""
DuckShootAlgoithm.py needs to be run from the command line
This code is for the remote-controlled The Duck Shoot PiWars 2018 challenge
http://piwars.org/
"""

# Import required libraries
import time
import logging
import math
import cwiid
import SetupConsoleLogger
import GPIOLayout
import ServoController
import SwitchingGPIO
import WiimoteNunchukControllerThread


# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER, logging.DEBUG)

# Servo used to fire the darts
servo_controller = ServoController.ServoController()

# Laser gpio line
laser_gpio = SwitchingGPIO.SwitchingGPIO(GPIOLayout.DUCK_SHOOT_LASER_GPIO)

# Motor gpio line
motor_gpio = SwitchingGPIO.SwitchingGPIO(GPIOLayout.DUCK_SHOOT_MOTOR_GPIO)

RUMBLE_DELAY = 2.0  # Time of rumble in seconds trigger has to go full length
NERF_TRIGGER_FORWARD = 45  # Angle of servo in forward position degrees
NERF_TRIGGER_BACK = -45  # Angle of servo in back position degrees


def fire_cb(currentWiimote):
    """
    thread rumble function
    """    
    LOGGER.info("Fire!!")
    currentWiimote.rumble = 1
    servo_controller.set_nerf_trigger_servo(NERF_TRIGGER_FORWARD)
    time.sleep(RUMBLE_DELAY)
    servo_controller.set_nerf_trigger_servo(NERF_TRIGGER_BACK)
    currentWiimote.rumble = 0


def toggle_laser_cb(currentWiimote):
    """
    thread rumble function
    """
    del currentWiimote

    if laser_gpio.is_on():
        LOGGER.info("Laser Toggle Off")
        laser_gpio.switch_off()
    else:
        LOGGER.info("Laser Toggle On")
        laser_gpio.switch_on()


def toggle_motor_cb(currentWiimote):
    """
    thread rumble function
    """
    del currentWiimote

    if motor_gpio.is_on():
        LOGGER.info("Motor Toggle Off")
        motor_gpio.switch_off()
    else:
        LOGGER.info("Motor Toggle On")
        motor_gpio.switch_on()


def main():
    """
    """
    LOGGER.info("Duck Shoot")
    servo_controller.set_nerf_trigger_servo(NERF_TRIGGER_BACK)
    WIIMOTE_CONTROLLER = WiimoteNunchukControllerThread.WiimoteNunchukControllerThread(
            fire_cb,
            toggle_laser_cb,
            toggle_motor_cb,
            servo_controller)
    try:
        WIIMOTE_CONTROLLER.start()
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        LOGGER.info("Stopping the Wiimote Controller thread")
    finally:
        WIIMOTE_CONTROLLER.exit_now()
        WIIMOTE_CONTROLLER.join()
        servo_controller.stop_servos()
        LOGGER.info("'Duck Shoot' Finished.")


if __name__ == "__main__":
    main()
