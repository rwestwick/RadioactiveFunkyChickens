#!/usr/bin/python
"""
WiimoteController.py needs to be run from the command line
This code is for the remote-controlled PiWars 2017 challenges:
Pi Noon
Obstacle Course
Skittles
Slightly Deranged Golf
http://piwars.org/

Need to install correct python modules, see
https://help.ubuntu.com/community/CWiiD
"""

# Import required libraries
import time
import logging
import MotorController
import SetupConsoleLogger
import GPIOLayout
import cwiid
import ServoController
import threading
import math
import SpeedSettings

# Set constants
STICK_DELAY = 0.1
BUTTON_DELAY = 0.1
RUMBLE_DELAY = 1

PAN_STEP = 10
TILT_STEP = 10

NUNCHUK_MAX = 220.0
NUNCHUK_MID = 126.0
NUNCHUK_MIN = 32.0
NUNCHUK_BUFFER = 25.0

# Define rumble function for thread


def rumble(currentLogger, currentWiimote):
    """thread rumble function"""
    currentLogger.info("Rumble")
    currentWiimote.rumble = 1
    time.sleep(RUMBLE_DELAY)
    currentWiimote.rumble = 0
    return

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
robotmove = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN, GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN, GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

# Initialise ServoController
SERVO_CONTROLLER = ServoController.ServoController()
SERVO_CONTROLLER.start_servos()


def main():
    """
    """
    # Set initial values
    speed = SpeedSettings.SPEED_FASTEST  # Initial forward speed
    tVal = 0  # 0 degrees is centre
    pVal = 0  # 0 degrees is centre

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

    # Respond to Nunchuk joystick
    while True:

        if 'nunchuk' in wm.state:
            # print("Success")

            buttons = wm.state['buttons']

            # X axis: Left Max = 25, Middle = 124, Right Max = 225
            NunchukStickX = (wm.state['nunchuk']['stick'][cwiid.X])
            # Y axis: Down Max = 30, Middle = 132, Up Max = 225
            NunchukStickY = (wm.state['nunchuk']['stick'][cwiid.Y])

            # print("Stick X: ", str(NunchukStickX),
            #       "Stick Y: ", str(NunchukStickY))

            # Go forward if joystick pushed forward beyond buffer in central
            # channel
            if (NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)
                    and NunchukStickX < (NUNCHUK_MID + NUNCHUK_BUFFER)
                    and NunchukStickX > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                speed = int(SpeedSettings.SPEED_FASTEST *
                            (NunchukStickY - NUNCHUK_MID) /
                            (NUNCHUK_MAX - NUNCHUK_MID))
                if speed > SpeedSettings.SPEED_FASTEST:
                    speed = SpeedSettings.SPEED_FASTEST
                robotmove.forward(speed)

                LOGGER.info("Forward at speed " + str(speed))
                time.sleep(STICK_DELAY)

            # Go backwards if joystick pulled back beyond buffer in central
            # channel
            elif (NunchukStickY < (NUNCHUK_MID - NUNCHUK_BUFFER)
                  and NunchukStickX < (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickX > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                speed = int(SpeedSettings.SPEED_FASTEST *
                            (NUNCHUK_MID - NunchukStickY) /
                            (NUNCHUK_MID - NUNCHUK_MIN))
                if speed > SpeedSettings.SPEED_FASTEST:
                    speed = SpeedSettings.SPEED_FASTEST
                robotmove.reverse(speed)

                LOGGER.info("Reverse at speed " + str(speed))
                time.sleep(STICK_DELAY)

            # Spin right right joystick pushed right beyond buffer in central
            # channel
            elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickY < (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickY > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                speed = int(SpeedSettings.SPEED_FASTEST *
                            (NunchukStickX - NUNCHUK_MID) /
                            (NUNCHUK_MAX - NUNCHUK_MID))
                if speed > SpeedSettings.SPEED_FASTEST:
                    speed = SpeedSettings.SPEED_FASTEST
                robotmove.spin_right(speed)

                LOGGER.info("Spin right at speed " + str(speed))
                time.sleep(STICK_DELAY)

            # Spin left if joystick pushed left beyond buffer in central
            # channel
            elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                  and NunchukStickY < (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickY > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                speed = int(SpeedSettings.SPEED_FASTEST *
                            (NUNCHUK_MID - NunchukStickX) /
                            (NUNCHUK_MID - NUNCHUK_MIN))
                if speed > SpeedSettings.SPEED_FASTEST:
                    speed = SpeedSettings.SPEED_FASTEST
                robotmove.spin_left(speed)

                LOGGER.info("Spin left at speed " + str(speed))
                time.sleep(STICK_DELAY)

            # Turn forward left if joystick pushed top left outside central
            # channels
            elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                  and NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)):

                # Calculate lenghts in range <100, min value depends on
                # NUNCHUK_BUFFER value
                lengthX = SpeedSettings.SPEED_FASTEST * \
                    (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                lengthY = SpeedSettings.SPEED_FASTEST * \
                    (NunchukStickY - NUNCHUK_MID) / (NUNCHUK_MAX - NUNCHUK_MID)

                # Speed is length of hypotenuse from Pythagoras
                overallSpeed = int(
                    math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))
                if overallSpeed > SpeedSettings.SPEED_FASTEST:
                    overallSpeed = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = int(lengthY)
                speedRightWheel = overallSpeed
                robotmove.turn_forward(speedLeftWheel, speedRightWheel)

                LOGGER.info(
                    "Steer left. Left wheel at speed: " + str(speedLeftWheel) +
                    " Right wheel at speed: " + str(speedRightWheel))
                time.sleep(STICK_DELAY)

            # Turn forward right if joystick pushed top right outside central
            # channels
            elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)):

                # Calculate lenghts in range <100, min value depends on
                # NUNCHUK_BUFFER value
                lengthX = SpeedSettings.SPEED_FASTEST * \
                    (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                lengthY = SpeedSettings.SPEED_FASTEST * \
                    (NunchukStickY - NUNCHUK_MID) / (NUNCHUK_MAX - NUNCHUK_MID)

                # Speed is length of hypotenuse from Pythagoras
                overallSpeed = int(
                    math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))
                if overallSpeed > SpeedSettings.SPEED_FASTEST:
                    overallSpeed = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = overallSpeed
                speedRightWheel = int(lengthY)
                robotmove.turn_forward(speedLeftWheel, speedRightWheel)

                LOGGER.info(
                    "Steer right. Left wheel at speed: " + str(speedLeftWheel)
                    + " Right wheel at speed: " + str(speedRightWheel))
                time.sleep(STICK_DELAY)

            # Turn reverse left if joystick pushed bottom left outside central
            # channels
            elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                  and NunchukStickY < (NUNCHUK_MID - NUNCHUK_BUFFER)):

                # Calculate lenghts in range <100, min value depends on
                # NUNCHUK_BUFFER value
                lengthX = SpeedSettings.SPEED_FASTEST * \
                    (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                lengthY = SpeedSettings.SPEED_FASTEST * \
                    (NUNCHUK_MID - NunchukStickY) / (NUNCHUK_MID - NUNCHUK_MIN)

                # Speed is length of hypotenuse from Pythagoras
                overallSpeed = int(
                    math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))
                if overallSpeed > SpeedSettings.SPEED_FASTEST:
                    overallSpeed = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = int(lengthY)
                speedRightWheel = overallSpeed
                robotmove.turn_reverse(speedLeftWheel, speedRightWheel)

                LOGGER.info(
                    "Reverse left. Left wheel at speed: " + str(speedLeftWheel)
                    + " Right wheel at speed: " + str(speedRightWheel))
                time.sleep(STICK_DELAY)

            # Turn reverse right if joystick pushed top right outside central
            # channels
            elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickY < (NUNCHUK_MID + NUNCHUK_BUFFER)):

                # Calculate lenghts in range <100, min value depends on
                # NUNCHUK_BUFFER value
                lengthX = SpeedSettings.SPEED_FASTEST * \
                    (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                lengthY = SpeedSettings.SPEED_FASTEST * \
                    (NUNCHUK_MID - NunchukStickY) / (NUNCHUK_MID - NUNCHUK_MIN)

                # Speed is length of hypotenuse from Pythagoras
                overallSpeed = int(
                    math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))
                if overallSpeed > SpeedSettings.SPEED_FASTEST:
                    overallSpeed = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = overallSpeed
                speedRightWheel = int(lengthY)
                robotmove.turn_reverse(speedLeftWheel, speedRightWheel)

                LOGGER.info("Reverse right. Left wheel at speed: " +
                            str(speedLeftWheel) + " Right wheel at speed: " +
                            str(speedRightWheel))
                time.sleep(STICK_DELAY)

            # else stop
            else:
                robotmove.stop()
                # LOGGER.info("Stop! X pos: " + str(NunchukStickX) +
                #             " Y pos: " + str(NunchukStickY))

                # Servo Controls

                # If button up pressed move servo up
            if (buttons & cwiid.BTN_UP):
                pVal = pVal + PAN_STEP
                if pVal > 90:
                    pVal = 90
                LOGGER.info("Servo Up to: " + str(pVal))
                SERVO_CONTROLLER.set_pan_servo(pVal)
                time.sleep(BUTTON_DELAY)

            # If button down pressed move servo down
            elif (buttons & cwiid.BTN_DOWN):
                pVal = pVal - PAN_STEP
                if pVal < -90:
                    pVal = -90
                LOGGER.info("Servo Down to: " + str(pVal))
                SERVO_CONTROLLER.set_pan_servo(pVal)
                time.sleep(BUTTON_DELAY)

            # If button right pressed move servo right
            elif (buttons & cwiid.BTN_RIGHT):
                tVal = tVal + TILT_STEP
                if tVal > 90:
                    tVal = 90
                LOGGER.info("Servo Right to: " + str(tVal))
                SERVO_CONTROLLER.set_tilt_servo(tVal)
                time.sleep(BUTTON_DELAY)

            # If button left pressed move servo left
            elif (buttons & cwiid.BTN_LEFT):
                tVal = tVal - TILT_STEP
                if tVal < -90:
                    tVal = -90
                LOGGER.info("Servo Left to: " + str(tVal))
                SERVO_CONTROLLER.set_tilt_servo(tVal)
                time.sleep(BUTTON_DELAY)

            # If botton A pressed centre Servo
            elif (buttons & cwiid.BTN_A):
                pVal = 0
                tVal = 0
                SERVO_CONTROLLER.set_pan_servo(pVal)
                SERVO_CONTROLLER.set_tilt_servo(tVal)
                LOGGER.info("Centre!")
                time.sleep(BUTTON_DELAY)

            # If button B pressed rumble Wiimote, but don't block other actions
            if (buttons & cwiid.BTN_B):
                t = threading.Thread(
                    target=rumble, args=(
                        LOGGER,
                        wm,
                    ))
                t.start()
                time.sleep(BUTTON_DELAY)

        # If no nunchuck detected yet, signal and wait
        else:
            print("Doh for now")
            time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the Wiimote Controller")
    finally:
        LOGGER.info("Wiimote Controller Finished")
        SERVO_CONTROLLER.stop_servos()
        robotmove.cleanup()
