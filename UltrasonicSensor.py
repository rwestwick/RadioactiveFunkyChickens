"""
"""

from __future__ import division # Used for floating point division in Python 2.7
import RPi.GPIO as GPIO
import logging
import Queue

module_logger = logging.getLogger("__main__.UltraonicSensor")


class UltraonicSensor():
    """
    """
    # Running average settings
    QSIZE = 8
    QINITIAL = 30.0 # Based on wall to wall distance of ~60cm

    def __init__(self, input_pin, output_pin):
        """
        """
        self.sonarInp = input_pin
        self.sonarOut = output_pin
        module_logger.info("Setting up UltraonicSensor Module (in:", input_pin, ", out:", output_pin, ")")

        # Use physical pin numbering
        GPIO.setmode(GPIO.BOARD)

        # Disable warnings
        GPIO.setwarnings(False)

        if self.sonarOut != self.sonarInp:
            # Initialise GPIO pins
            GPIO.setup(self.sonarOut, GPIO.OUT)
            GPIO.setup(self.sonarInp,GPIO.IN)

        # Initilise Queue
        self.q = Queue.Queue()

        for i in xrange(0,QSIZE):
            self.q.put(QINITIAL)

    def Measurement(self):    # Returns the distance in cm to the nearest reflecting object
        """
        """
        # If the two pins are actually the same, then
        # they need to be switched between input and
        # output
        if self.sonarOut == self.sonarInp:
            GPIO.setupself.sonarOut, GPIO.OUT)

        # Send 10us pulse to trigger
        GPIO.output(self.sonarOut, True)
        time.sleep(0.00001)
        GPIO.output(self.sonarOut, False)
        start = time.time()
        count=time.time()

        # If the two pins are actually the same, then
        # they need to be switched between input and
        # output
        if self.sonarOut == self.sonarInp:
            GPIO.setup(self.sonarInp, GPIO.IN)

        # Measure echo
        while GPIO.input(self.sonarInp)==0 and time.time()-count<0.1:
            start = time.time()
        count=time.time()
        stop=count
        while GPIO.input(self.sonarInp)==1 and time.time()-count<0.1:
            stop = time.time()

        # Calculate pulse length
        elapsed = stop - start

        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound 34000(cm/s) divided by 2
        distance = elapsed * 17000

        # Add latest distance to queue
        if (self.q.qsize() > QSIZE) :
            self.q.get()
        self.q.put(distance)

        # Calculate running average
        total = 0
        for val in self.q.queue:
            total = total + val

        return total / self.q.qsize()


if __name__ == "__main__":
    try:
        # UltraSonic physical pin numbers
        sonarInpRight = 16 # Connected to Echo
        sonarOutRight = 31 # Connected to Trig

        sonarInpLeft = 15 # Connected to Echo
        sonarOutLeft = 12 # Connected to Trig
        
        # Define Sonar Pin (Uses same pin for both Ping and Echo)
        sonarSingleIO = 38

        UltraonicSensor proxity_two_io_left(sonarInpLeft, sonarOutLeft)
        print("proxity_two_io_left: ", proxity_two_io_left.Measurement())
        
        UltraonicSensor proxity_two_io_right(sonarInpRight, sonarOutRight)
        print("proxity_two_io_right: ", proxity_two_io_right.Measurement())

        UltraonicSensor proxity_one_io(sonarSingleIO, sonarSingleIO)
        print("proxity_one_io: ", proxity_one_io.Measurement())
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
