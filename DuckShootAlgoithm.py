#!/usr/bin/python
"""
Provides an mechanism for competing in the duck shoot stage of the competition.
"""

import logging
import time
import MotorController
import SwitchingGPIO
import SetupConsoleLogger
import GPIOLayout
import KeyboardCharacterReader
import cwiid
import SpeedSettings

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
GUNMOUNT = MotorController.MotorController(
    GPIOLayout.MOTOR_DRIVE_FORWARD_PIN, GPIOLayout.MOTOR_DRIVE_BACKWARD_PIN,
    GPIOLayout.MOTOR_ELEVATION_PIN, GPIOLayout.MOTOR_DECLINATION_PIN)

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN, GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN, GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

LASER = SwitchingGPIO(GPIOLayout.DUCK_SHOOT_LASER)

FIRE = SwitchingGPIO(GPIOLayout.DUCK_SHOOT_FIRE)

STICK_DELAY = 0.1

LASER_ON = False

# Set initial values
ELEVATION_SPEED = SpeedSettings.SPEED_SLOW


def wmMove(wm, robotmove):
    """
    Called to move the robot using the numchuck controller
    """
    speed = SpeedSettings.SPEED_FASTEST  # Initial forward speed

    if 'nunchuk' in wm.state:
        # X axis: Left Max = 25, Middle = 125, RightMax = 225
        NunchukStickX = (wm.state['nunchuk']['stick'][cwiid.X])
        # Y axis: DownMax = 30, Middle = 125, UpMax = 225
        NunchukStickY = (wm.state['nunchuk']['stick'][cwiid.Y])

        # Go forward if joystick pushed forward
        if (NunchukStickY > 150) & (NunchukStickY < 190):
            speed = SpeedSettings.SPEED_SLOW
            robotmove.forward(speed)
            LOGGER.info("Forward at speed " + str(speed))
            time.sleep(STICK_DELAY)

        elif (NunchukStickY >= 190):
            speed = SpeedSettings.SPEED_FASTEST
            robotmove.forward(speed)
            LOGGER.info("Forward at speed " + str(speed))
            time.sleep(STICK_DELAY)

        # Go backwards if joystick pulled back
        elif (NunchukStickY < 100) & (NunchukStickY > 50):
            speed = SpeedSettings.SPEED_SLOW
            robotmove.reverse(speed)
            LOGGER.info("Reverse at speed " + str(speed))
            time.sleep(STICK_DELAY)

        elif (NunchukStickY <= 50):
            speed = SpeedSettings.SPEED_FASTEST
            robotmove.reverse(speed)
            LOGGER.info("Reverse at speed " + str(speed))
            time.sleep(STICK_DELAY)

        # Spin right right joystick pushed right
        elif (NunchukStickX > 150) & (NunchukStickX < 190):
            speed = SpeedSettings.SPEED_SLOW
            robotmove.spin_right(speed)
            LOGGER.info("Spin right at speed " + str(speed))
            time.sleep(STICK_DELAY)

        elif (NunchukStickX >= 190):
            speed = SpeedSettings.SPEED_FASTEST
            robotmove.spin_right(speed)
            LOGGER.info("Spin right at speed " + str(speed))
            time.sleep(STICK_DELAY)

        # Spin left if joystick pushed left
        elif (NunchukStickX < 100) & (NunchukStickX > 50):
            speed = SpeedSettings.SPEED_SLOW
            robotmove.spin_left(speed)
            LOGGER.info("Spin left at speed " + str(speed))
            time.sleep(STICK_DELAY)

        elif (NunchukStickX <= 50):
            speed = SpeedSettings.SPEED_FASTEST
            robotmove.spin_left(speed)
            LOGGER.info("Spin left at speed " + str(speed))
            time.sleep(STICK_DELAY)

        # else stop
        else:
            robotmove.stop()
            LOGGER.info("Stop!")


def keyboardMovementsOfGun(keyp, gunmove):
    """
    Performs the main duck shooting algorithm
    """

    # Increase height of gun
    if keyp == '>':
        gunmove.backward(ELEVATION_SPEED)
        time.sleep(0.5)
        gunmove.stop()

    # Decrease height of gun
    elif keyp == '<':
        gunmove.forward(ELEVATION_SPEED)
        time.sleep(0.5)
        gunmove.stop()

    # fire (enter) is pressed
    elif keyp == ord('\n'):
        LASER.switch_on()
        time.sleep(1)
        LASER.switch_off()

    # laser (enter) is pressed
    elif keyp == 'l':
        if LASER_ON is True:
            LASER.switch_off()
        else:
            LASER.switch_on()
        time.sleep(0.5)

    # To end if Ctrl-C pressed
    elif ord(keyp) == 3:
        LOGGER.info("Break control loop!")
        return False

    return True


def main():
    """
    Performs the main duck shooting algorithm
    """

    # Connecting to the wiimote. This allows several attempts
    # as first few often fail.
    LOGGER.info("Press 1+2 on your Wiimote now ...")
    time.sleep(1)

    wm = None
    i = 2
    while not wm:
        try:
            wm = cwiid.Wiimote()
        except RuntimeError:
            if (i > 5):
                LOGGER.info("Cannot create Wiimote connection.")
                quit()
            LOGGER.info("Error opening wiimote connection, attempt " + str(i))
            i += 1

    LOGGER.info("Wiimote connected.")

    # Set wiimote to report button presses
    wm.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_EXT

    # Turn on led to show connected
    wm.led = 1

    # Give remote control keys
    print "Steer robot by using the numchuck"
    print "Use , or < to lower gun"
    print "Use . or > to raise gun"
    print "Use Enter to fire"
    print "Use l to toggle the laser pointer"
    print "Press Ctrl-C to end"

    # Respond to key presses
    while keyboardMovementsOfGun(KeyboardCharacterReader.readkey(), GUNMOUNT):

        # Perform the movement via the numchuck
        wmMove(wm, GUNMOUNT)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the Duck Shoot")
    finally:
        LOGGER.info("Duck Shoot Finished")
        GUNMOUNT.cleanup()
