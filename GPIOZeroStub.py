#!/usr/bin/python
"""
Provides a test version of the GPIOZero.GPIODevice class for platforms where
it doesn't exist.
"""


class OutputDevice(object):
    """
    Provides a stub for the GPIODevice class defined in GPIOZero
    """

    def __init__(self, param1):
        """
        Stub function
        """

    # pylint: disable=C0103
    def on(self):
        """
        Stub function
        """

    def off(self):
        """
        Stub function
        """

    def close(self):
        """
        Stub function
        """

    def is_active(self):
        """
        Stub function
        """
