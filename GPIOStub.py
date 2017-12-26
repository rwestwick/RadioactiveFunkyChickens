#!/usr/bin/python
"""
Provides a test version of the GPIO class for platforms where
it doesn't exist.
"""

from random import *

BOARD = 1
OUT = 2
IN = 3


class pwmObject:
    """
    Provides a stub for the PWM class defined in Rpi
    """

    def __init__(self):
        """
        Stub function
        """

    def start(self, param):
        """
        Stub function
        """

    def ChangeDutyCycle(self, param1):
        """
        Stub function
        """


def setmode(param):
    """
    Stub function
    """


def setwarnings(param):
    """
    Stub function
    """


def setup(param1, param2):
    """
    Stub function
    """


def output(param1, param2):
    """
    Stub function
    """


def input(param1):
    """
    Stub function
    """
    return randint(0, 1)


def PWM(param1, param2):
    """
    Stub function
    """
    return pwmObject()


def cleanup():
    """
    Stub function
    """
