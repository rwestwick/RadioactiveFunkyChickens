#!/usr/bin/python

# Import libraries
import RPi.GPIO as GPIO
import time

# Reading single character by forcing stdin to raw mode
import sys
import tty
import termios

# Pin numbers are in BCM
# Tilting is raising or lowering their head to look up or down.
# Panning is spinning left or right.
PAN_SERVO_PIN = 25
TILT_SERVO_PIN = 24

PWM_FREQ = 50.0  # Hz gives pulses at 20ms
# PWM_FREQ = 200.0 # Hz gives pulses at 5ms - N.B. Jitter worse at higher frequencies
# PWM_FREQ = 500.0 # Hz gives pulses at 2ms - This is the value used in
# PiRoCon this though will not give pulse width of 2.5ms
PERIOD = 1000.0 / PWM_FREQ  # ms
MIN_PULSE_WIDTH = 0.5  # ms for 0 deg
MAX_PULSE_WIDTH = 2.5  # ms for 180 deg
DC_ZERO_DEG = (MIN_PULSE_WIDTH / PERIOD) * \
    100.0  # Duty Cycle % for zero degrees
# Duty Cycle % change from zero degrees to 180 degrees at 50 Hz
DC_PLUS180_SWING = ((MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) / PERIOD) * 100.0


def dutyCycle(angle):
    """
    Calculates the Duty Cycle for the servo
    """
    dutyCycle = ((angle / 180.0) * DC_PLUS180_SWING) + DC_ZERO_DEG

    return dutyCycle


def setServo(servo, degrees):
    """
    Sets servo's Duty cycle to move servo to degrees
    """
    dc = dutyCycle(degrees)
    servo.ChangeDutyCycle(dc)


def startServos():
    """
    Initialises the PIN settings for controlling pan and tilt servos
    """

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(PAN_SERVO_PIN, GPIO.OUT)
    GPIO.setup(TILT_SERVO_PIN, GPIO.OUT)

    pwmPan = GPIO.PWM(PAN_SERVO_PIN, PWM_FREQ)
    pwmTilt = GPIO.PWM(TILT_SERVO_PIN, PWM_FREQ)

    dcPan = dutyCycle(0.0)
    dcTilt = dutyCycle(0.0)

    pwmPan.start(dcPan)
    pwmTilt.start(dcTilt)

    return [pwmPan, pwmTilt]


def stopServos(servo1, servo2):
    """
    Stops the servos by stopping PWM and cleaning up GPIO
    """
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()


def readchar():
    """

    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    if ch == '0x03':
        raise KeyboardInterrupt
    return ch


def readkey(getchar_fn=None):
    """

    """
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)  # 16=Up, 17=Down, 18=Right, 19=Left arrows


if __name__ == "__main__":
    try:
        # Starts the servos using RPi.GPIO
        [pwmPan, pwmTilt] = startServos()
        time.sleep(1)
        # Test basic functionality

        print "Testing basic functionality"

        pVal = 0.0  # degs
        tVal = 0.0  # degs
        setServo(pwmPan, pVal)
        setServo(pwmTilt, tVal)

        time.sleep(1)

        pVal = 90.0  # degs
        tVal = 90.0  # degs
        setServo(pwmPan, pVal)
        setServo(pwmTilt, tVal)

        time.sleep(1)

        pVal = 180.0  # degs
        tVal = 180.0  # degs
        setServo(pwmPan, pVal)
        setServo(pwmTilt, tVal)

        time.sleep(1)

        pVal = 90.0  # degs
        tVal = 90.0  # degs
        setServo(pwmPan, pVal)
        setServo(pwmTilt, tVal)

        time.sleep(1)

        print "Use Arrows or W-Up, Z-Down, A-Left, S-Right Space=Centre, ^C=Exit:"

        # Control servos using Key presses
        while True:
            key = readkey()
            if key == ' ':
                tVal = 90.0
                pVal = 90.0
                setServo(pwmPan, pVal)
                setServo(pwmTilt, tVal)
                print "Centre", tVal, pVal

            elif key.upper() == 'L':
                tVal = 0.0
                pVal = 0.0
                setServo(pwmPan, pVal)
                setServo(pwmTilt, tVal)
                print "Left", tVal, pVal

            elif key.upper() == 'R':
                tVal = 180.0
                pVal = 180.0
                setServo(pwmPan, pVal)
                setServo(pwmTilt, tVal)
                print "Right", tVal, pVal

            elif key == ' x' or key == '.':
                stopServos(pwmPan, pwmTilt)
                print "Stop"

            elif key == 'w' or ord(key) == 16:  # 16=Up arrow
                tVal = max(0.0, tVal - 10.0)
                setServo(pwmTilt, tVal)
                print "Up", tVal

            elif key == 'a' or ord(key) == 19:  # 19=Left arrow
                pVal = max(0.0, pVal - 10.0)
                setServo(pwmPan, pVal)
                print "Left", pVal

            elif key == 's' or ord(key) == 18:  # 18=Right arrow
                pVal = min(180.0, pVal + 10.0)
                setServo(pwmPan, pVal)
                print "Right", pVal

            elif key == 'z' or ord(key) == 17:  # 17=Down arrow
                tVal = min(180.0, tVal + 10.0)
                setServo(pwmTilt, tVal)
                print "Down", tVal

            elif ord(key) == 3:
                break

    except KeyboardInterrupt:
        print("Ending session.")

    finally:
        stopServos(pwmPan, pwmTilt)
