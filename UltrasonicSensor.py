#!/usr/bin/python

"""
Class defines how to interract with the Ultrasonic sensor
"""

# Used for floating point division in Python 2.7
from __future__ import division
import RPi.GPIO as GPIO
import logging
import Queue
import time

MODULE_LOGGER = logging.getLogger("__main__.UltrasonicSensor")


class UltrasonicSensor(object):  # pylint: disable=too-few-public-methods

    """
    Class defines how to interract with the Ultrasonic sensor
    """
    # Running average settings
    QSIZE = 8
    QINITIAL = 30.0  # Based on wall to wall distance of ~60cm

    def __init__(self, input_pin, output_pin=None):
        """
        """
        self.sonar_in = input_pin
        if output_pin is not None:
            self.sonar_out = output_pin
        else:
            self.sonar_out = input_pin

        log_string = "Setting up UltraonicSensor Module (in:" + \
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

        for _ in xrange(0, self.QSIZE):
            self.queue.put(self.QINITIAL)

    def measurement(self):
        """
        Returns the distance in cm to the nearest reflecting object
        """
        # If the two pins are actually the same, then
        # they need to be switched between input and
        # output
        if self.sonar_out == self.sonar_in:
            GPIO.setup(self.sonar_out, GPIO.OUT)

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
        for val in self.queue.queue:
            total = total + val

        return total / self.queue.qsize()


if __name__ == "__main__":
    try:
        # UltraSonic physical pin numbers
        SONAR_INPUT_RIGHT = 16  # Connected to Echo
        SONAR_OUTPUT_RIGHT = 31  # Connected to Trig

        SONAR_INPUT_LEFT = 15  # Connected to Echo
        SONAR_OUTPUT_LEFT = 12  # Connected to Trig

        # Define Sonar Pin (Uses same pin for both Ping and Echo)
        SONAR_SINGLE_IO = 38

        PROXITY_TWO_IO_LEFT = UltrasonicSensor(SONAR_INPUT_LEFT,
                                               SONAR_OUTPUT_LEFT)
        print("PROXITY_TWO_IO_LEFT: ", PROXITY_TWO_IO_LEFT.measurement())

        PROXITY_TWO_IO_RIGHT = UltrasonicSensor(SONAR_INPUT_RIGHT,
                                                SONAR_OUTPUT_RIGHT)
        print("PROXITY_TWO_IO_RIGHT: ", PROXITY_TWO_IO_RIGHT.measurement())

        PROXITY_ONE_IO = UltrasonicSensor(SONAR_SINGLE_IO)
        print("PROXITY_ONE_IO: ", PROXITY_ONE_IO.measurement())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
