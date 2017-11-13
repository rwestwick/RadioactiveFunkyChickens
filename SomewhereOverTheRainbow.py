#!/usr/bin/python
# SomewhereOverTheRainbow.pw
# http://piwars.org/2018-competition/challenges/somewhere-over-the-rainbow/

"""
This algorithm is designed to solve the Somewhere Over the Rainbow Pi Wars 4.0
challenge.
"""

import logging
import time
import SetupConsoleLogger
import ServoController
import MotorController
import UltrasonicSensor
import GPIOLayout
import KeyboardCharacterReader
import RPi.GPIO as GPIO

# Create a logger to both file and stdout
LOGGER = logging.getLogger("__name__")
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set initial constant values
FRONT_BUFFER_WARN = 35 # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 25 # Shortest distance to front (cm)
SIDE_BUFFER = 10 # Shortest distance to side (cm)
CORRECTION_TIME = 0.15 # Angle correction delay time in seconds
FORWARD_TIME = 0.1 # Angle correction delay time in seconds
TURN_DELAY = 0.65

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

# Initialise servos
SERVO_CONTROLLER = ServoController.ServoController()

def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    Method 1 - First choice
    Method 2 - Emergency backup
    """

    LOGGER.info("Minimal Maze")

    # Waiting for start of challenge
    LOGGER.info("To start 'Somewhere Over the Rainbow' press 'Space' key.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_PIN,
        GPIOLayout.SONAR_LEFT_TX_PIN)
    view_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN,
        GPIOLayout.SONAR_RIGHT_TX_PIN)
    view_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_PIN)

    LOGGER.info("Distance view_left at start " + format(view_left.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_right at start " + format(view_right.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_front at start " + format(view_front.measurement(), '.2f') + " cm")

    # Setting the camera to look ahead
    SERVO_CONTROLLER.start_servos()
    time.sleep(1) # Need to add a delay or echo does not work!
    SERVO_CONTROLLER.set_pan_servo(0)
    SERVO_CONTROLLER.set_tilt_servo(0)
    time.sleep(1)

    # Start the challenge
    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    # Capture live video feed

    # Spin robot to work out position of each coloured marker

    # Point towards Red marker and go into quater cirle zone

    # Point towards Blue marker and go into quater cirle zone

    # Point towards Yellow marker and go into quater cirle zone

    # Point towards Green marker and go into quater cirle zone

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Somewhere Over the Rainbow'.")
    finally:
        LOGGER.info("'Somewhere Over the Rainbow' Finished.")
        SERVO_CONTROLLER.stop_servos()
        ROBOTMOVE.cleanup()
        GPIO.cleanup()
        
        
