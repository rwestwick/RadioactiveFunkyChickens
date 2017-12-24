#!/usr/bin/python

"""
Class defines how to interract with the Ultrasonic sensor
"""

# Used for floating point division in Python 2.7
from __future__ import division
import Queue
import time
import logging
import RPi.GPIO as GPIO
import SetupConsoleLogger
import GPIOLayout

MODULE_LOGGER = logging.getLogger("__main__.UltrasonicSensor")


class UltrasonicSensor(object):  # pylint: disable=too-few-public-methods

    """
    Class defines how to interract with the Ultrasonic sensor
    """

    def __init__(self, input_pin, output_pin=None):
        """
        Initialises the class
        """

        # Running average settings
        self.QSIZE = 1

        GPIO.setwarnings(False)

        self.sonar_in = input_pin
        if output_pin is not None:
            self.sonar_out = output_pin
        else:
            self.sonar_out = input_pin

        log_string = "Setting up UltrasonicSensor Module (in:" + \
            str(input_pin) + \
            ", out:" + \
            str(output_pin) + \
            ")"
        MODULE_LOGGER.info(log_string)

        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)

        if self.sonar_out != self.sonar_in:
            # Initialise GPIO pins
            GPIO.setup(self.sonar_out, GPIO.OUT)
            GPIO.setup(self.sonar_in, GPIO.IN)

        # Initilise Queue
        self.queue = Queue.Queue()

    def measurement(self):
        """
        Returns the distance in cm to the nearest reflecting object
        """
        self.queue = Queue.Queue()

        for x in range(0, self.QSIZE):
            # If the two pins are actually the same, then
            # they need to be switched between input and
            # output
            if self.sonar_out == self.sonar_in:
                GPIO.setup(self.sonar_out, GPIO.OUT)

            # Set trigger to False (Low)
            GPIO.output(self.sonar_out, False)

            # Allow module to settle
            time.sleep(0.1)

            # Send 10us pulse to trigger
            GPIO.output(self.sonar_out, True)
            time.sleep(0.00001)
            GPIO.output(self.sonar_out, False)
            start = time.time()
            count = time.time()

            # If the two pins are actually the same, then
            # they need to be switched between input and
            # output
            if self.sonar_out == self.sonar_in:
                GPIO.setup(self.sonar_in, GPIO.IN)

            # Measure echo
            while GPIO.input(self.sonar_in) == 0 and time.time() - count < 0.1:
                start = time.time()
            count = time.time()
            stop = count
            while GPIO.input(self.sonar_in) == 1 and time.time() - count < 0.1:
                stop = time.time()

            # Calculate pulse length
            elapsed = stop - start

            # Distance pulse travelled in that time is time
            # multiplied by the speed of sound 34000(cm/s) divided by 2
            distance = elapsed * 17000

            # Add latest distance to queue
            if self.queue.qsize() > self.QSIZE:
                self.queue.get()
            self.queue.put(distance)

        # Calculate running average
        total = 0
        stringvals = ""
        for val in self.queue.queue:
            stringvals += format(val, '.2f') + " : "
            total = total + val

        #stringvals += "avg " + format((total / self.queue.qsize()), '.2f')
        MODULE_LOGGER.info("Ultrasonic values: " + stringvals)

        return total / self.queue.qsize()


if __name__ == "__main__":
    try:
        SetupConsoleLogger.setup_console_logger(MODULE_LOGGER)

        PROXITY_TWO_IO_LEFT = UltrasonicSensor(GPIOLayout.SONAR_LEFT_RX_PIN,
                                               GPIOLayout.SONAR_LEFT_TX_PIN)
        for x in range(0, PROXITY_TWO_IO_LEFT.QSIZE * 2):
            PROXITY_TWO_IO_LEFT.measurement()

        MODULE_LOGGER.info("PROXITY_TWO_IO_LEFT: " +
                           str(PROXITY_TWO_IO_LEFT.measurement()))

        PROXITY_TWO_IO_RIGHT = UltrasonicSensor(GPIOLayout.SONAR_RIGHT_RX_PIN,
                                                GPIOLayout.SONAR_RIGHT_TX_PIN)
        for x in range(0, PROXITY_TWO_IO_RIGHT.QSIZE * 2):
            PROXITY_TWO_IO_RIGHT.measurement()

        MODULE_LOGGER.info("PROXITY_TWO_IO_RIGHT: " +
                           str(PROXITY_TWO_IO_RIGHT.measurement()))

        PROXITY_ONE_IO = UltrasonicSensor(GPIOLayout.SONAR_FRONT_TX_PIN)
        for x in range(0, PROXITY_ONE_IO.QSIZE * 2):
            PROXITY_ONE_IO.measurement()

        MODULE_LOGGER.info("PROXITY_ONE_IO: " +
                           str(PROXITY_ONE_IO.measurement()))
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
