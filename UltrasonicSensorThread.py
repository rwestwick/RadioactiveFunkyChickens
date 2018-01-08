#!/usr/bin/python
"""
Class defines how to interact with an Ultrasonic sensor via a thread
"""

# Used for floating point division in Python 2.7
from __future__ import division
import time
import logging
import threading
import UltrasonicSensor

MODULE_LOGGER = logging.getLogger("__main__.UltrasonicSensorThread")


class UltrasonicSensorThread(threading.Thread):
    """
    Defines the interaction with a temperature sensor
    """
    DEFAULT_TEMP = -42.0

    def __init__(self, delay, callback, input_pin, output_pin=None, qsize=1):
        """
        Initialise the parameters required for UltrasonicSensorThread
        """
        self._distance = 0.0
        self._delay = delay
        self._mutex = threading.Lock()
        self._exit_now = False
        self._callback = callback
        self._sensor = UltrasonicSensor.UltrasonicSensor(
            input_pin, output_pin, qsize)

        MODULE_LOGGER.info("Setting up UltrasonicSensorThread Module")

        threading.Thread.__init__(self)

    def __del__(self):
        """
        Destructor
        """
        self.exit_now()
        MODULE_LOGGER.debug("Destructor")

    def __store_data(self, params):
        """
        Stores the data via a mutex in the class variable
        """
        self._mutex.acquire()
        try:
            # store_data
            self._distance = params
            if self._callback is not None:
                self._callback(params)
        finally:
            self._mutex.release()

    def __get_distance(self):
        """
        Returns the temperature that has been read
        """
        distance = self._sensor.measurement()
        MODULE_LOGGER.debug('Timed Reading Temp={0:0.2f}*C '.format(distance))
        return distance

    def read_data(self):
        """
        Retrieves the data via a mutex from the class variable
        """
        MODULE_LOGGER.debug("Reading data")
        distance = 0
        self._mutex.acquire()
        try:
            # store_data
            distance = self._distance
        finally:
            self._mutex.release()
        return distance

    def exit_now(self):
        """
        Request the thread to exit
        """
        MODULE_LOGGER.debug("Request to exit")
        self._exit_now = True

    def run(self):
        """
        Performs the sensor reading from within the thread
        """
        MODULE_LOGGER.debug("Starting sensor reading thread")
        while not self._exit_now:
            MODULE_LOGGER.info("timer fired")
            self.__store_data(self.__get_distance())
            time.sleep(self._delay)
        MODULE_LOGGER.debug("Finished thread")
