#!/usr/bin/python
"""
Provides a test version of the GPIO class for platforms where
it doesn't exist.
"""

import random

BOARD = 1
OUT = 2
IN = 3


class pwmObject(object):
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
        del param

    def ChangeDutyCycle(self, param1):
        """
        Stub function
        """
        del param1


def setmode(param):
    """
    Stub function
    """
    del param


def setwarnings(param):
    """
    Stub function
    """
    del param


def setup(param1, param2):
    """
    Stub function
    """
    del param2


def output(param1, param2):
    """
    Stub function
    """
    del param2


def input(param1):
    """
    Stub function
    """
    del param1
    return random.randint(0, 1)


def PWM(param1, param2):
    """
    Stub function
    """
    del param1, param2
    return pwmObject()


def cleanup():
    """
    Stub function
    """
