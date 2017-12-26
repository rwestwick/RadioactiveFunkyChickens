#!/usr/bin/python
"""
Class defines how to interract with the Ultrasonic sensor
"""

# Used for floating point division in Python 2.7
from __future__ import division
import Queue
import time
import logging
import platform
if platform.machine() == "armv6l" or platform.machine() == "armv7l":
    import RPi.GPIO as GPIO
else:
    import GPIOStub as GPIO

MODULE_LOGGER = logging.getLogger("__main__.UltrasonicSensor")


class UltrasonicSensor(object):  # pylint: disable=too-few-public-methods
    """
    Class defines how to interract with the Ultrasonic sensor
    """

    def __init__(self, input_pin, output_pin=None, qsize=1):
        """
        Initialises the class
        """

        # Running average settings
        self.qsize = qsize

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

        for num_readings in range(0, self.qsize):
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
            if self.queue.qsize() >= self.qsize:
                self.queue.get()
            self.queue.put(distance)

        # Calculate running average
        total = 0
        stringvals = ""
        for val in self.queue.queue:
            stringvals += format(val, '.2f') + " : "
            total = total + val

        MODULE_LOGGER.info("Ultrasonic values: " + stringvals)

        return total / self.queue.qsize()

    def cleanup(self):
        """
        clears down the GPIO
        """
        GPIO.cleanup()
