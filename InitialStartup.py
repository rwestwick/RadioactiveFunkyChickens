#!/usr/bin/python
"""
This code is used by Raspian on startup to initialise the various devices
used on the robot.
 - PiWars 2017 challenge http://piwars.org/
"""

import platform
import logging
import time
import DualMotorController
import UltrasonicSensor
import SetupConsoleLogger
import ServoController
import SwitchingGPIO
import GPIOLayout
if platform.machine() == "armv6l" or platform.machine() == "armv7l":
    import RPi.GPIO as GPIO
else:
    import GPIOStub as GPIO


# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)


def initialise_servos():
    """
    Initialise the three servos to their 0 degree position
    """
    LOGGER.info("  Initialising Servos")

    servo_controller = ServoController.ServoController()
    servo_controller.start_servos()
    time.sleep(1)
    servo_controller.set_nerf_trigger_servo(0)
    servo_controller.set_pan_servo(0)
    servo_controller.set_tilt_servo(0)
    time.sleep(1)
    servo_controller.stop_servos()


def initialise_motor_controllers():
    """
    Initialise the motor controllers to outputs and all "off"
    """
    LOGGER.info("  Initialising Motor Controllers")

    motor_controller = DualMotorController.DualMotorController(
        GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
        GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
        GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
        GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
        GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
        GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
        GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
        GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)
    motor_controller.stop()
    motor_controller.cleanup()


def initialise_ultrasonics():
    """
    Initialise the ultrasonics to inputs and outputs and off if required
    """
    LOGGER.info("  Initialising Ultrasonics")

    prox_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_GPIO)
    prox_left = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_GPIO, GPIOLayout.SONAR_LEFT_TX_GPIO)
    prox_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_GPIO, GPIOLayout.SONAR_RIGHT_TX_GPIO)
    prox_front.cleanup()
    prox_left.cleanup()
    prox_right.cleanup()


def initialise_other_gpio():
    """
    Initialise the gpio (laser, motor etc) to outputs and off
    """
    LOGGER.info("  Initialising GPIO's")

    laser_gpio = SwitchingGPIO.SwitchingGPIO(GPIOLayout.DUCK_SHOOT_LASER_GPIO,
                                             False)
    laser_gpio.switch_off()
    motor_gpio = SwitchingGPIO.SwitchingGPIO(GPIOLayout.DUCK_SHOOT_MOTOR_GPIO,
                                             False)
    motor_gpio.switch_off()


def main():
    """
    Main function to call each of the initialisation routines
    """
    initialise_servos()
    initialise_motor_controllers()
    initialise_ultrasonics()
    initialise_other_gpio()


if __name__ == "__main__":
    try:
        LOGGER.info("Starting the robot intialisation routine")
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the robot intialisation routine")
    finally:
        LOGGER.info("Completed the robot intialisation routine")
