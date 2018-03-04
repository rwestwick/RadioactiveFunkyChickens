#!/usr/bin/python
"""
DuckShootAlgoithm.py needs to be run from the command line
This code is for the remote-controlled The Duck Shoot PiWars 2018 challenge
http://piwars.org/

Need to install correct python modules, see
https://help.ubuntu.com/community/CWiiD
"""

# Import required libraries
import time
import logging
import DualMotorController
import SetupConsoleLogger
import GPIOLayout
import cwiid
import ServoController
import threading
import math
import SpeedSettings
import SwitchingGPIO


# Set constants
STICK_DELAY = 0.1  # seconds
BUTTON_DELAY = 0.1  # seconds
RUMBLE_DELAY = 2.0  # Time of rumble in seconds trigger has to go full length
NERF_TRIGGER_BACK = -45  # Angle of servo in back position degrees
NERF_TRIGGER_FORWARD = 45  # Angle of servo in forward position degrees

PAN_STEP = 10
TILT_STEP = 10

NUNCHUK_MAX = 220.0
NUNCHUK_MID = 126.0
NUNCHUK_MIN = 32.0
NUNCHUK_BUFFER = 25.0

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
robotmove = DualMotorController.DualMotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_PIN,
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_PIN)

# Initialise ServoController
SERVO_CONTROLLER = ServoController.ServoController()
SERVO_CONTROLLER.start_servos()

# Laser Status
LASER_ON = False
LASER_TOGGLE = SwitchingGPIO.SwitchingGPIO(GPIOLayout.DUCK_SHOOT_LASER)

# Motor Status
MOTOR_ON = False
MOTOR_TOGGLE = SwitchingGPIO.SwitchingGPIO(GPIOLayout.DUCK_SHOOT_MOTOR)


def rumble(currentLogger, currentWiimote):
    """
    thread rumble function
    """
    currentLogger.info("Rumble")
    SERVO_CONTROLLER.set_nerf_trigger_servo(NERF_TRIGGER_FORWARD)
    currentWiimote.rumble = 1
    time.sleep(RUMBLE_DELAY)
    SERVO_CONTROLLER.set_nerf_trigger_servo(NERF_TRIGGER_BACK)
    currentWiimote.rumble = 0
    return


def ToggleLaser(currentLogger):
    """
    thread rumble function
    """
    if LASER_ON:
        currentLogger.info("Laser Toggle Off")
        LASER_TOGGLE.switch_off()
        LASER_ON = False
    else:
        currentLogger.info("Laser Toggle On")
        LASER_TOGGLE.switch_on()
        LASER_ON = True
    return


def ToggleMotor(currentLogger):
    """
    thread rumble function
    """
    if MOTOR_ON:
        currentLogger.info("Motor Toggle Off")
        MOTOR_ON_TOGGLE.switch_off()
        MOTOR_ON = False
    else:
        currentLogger.info("Motor Toggle On")
        MOTOR_ON_TOGGLE.switch_on()
        MOTOR_ON = True
    return


def main():
    """
    """
    # Set initial values
    speed = SpeedSettings.SPEED_FASTEST  # Initial forward speed
    tVal = 0  # 0 degrees is centre
    pVal = 0  # 0 degrees is centre
    SERVO_CONTROLLER.set_nerf_trigger_servo(NERF_TRIGGER_BACK)
    

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

                LOGGER.info("Spin left at speed " + str(speed))

                robotmove.spin_left(speed)
                time.sleep(STICK_DELAY)

            # Turn forward left if joystick pushed top left outside central
            # channels
            elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                  and NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)):

                # Calculate lengths in range <100, min value depends on
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
                if lengthY > SpeedSettings.SPEED_FASTEST:
                    lengthY = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = int(lengthY)
                speedRightWheel = overallSpeed

                LOGGER.info(
                    "Steer left. Left wheel at speed: " + str(speedLeftWheel) +
                    " Right wheel at speed: " + str(speedRightWheel))

                robotmove.turn_forward(speedLeftWheel, speedRightWheel)
                time.sleep(STICK_DELAY)

            # Turn forward right if joystick pushed top right outside central
            # channels
            elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)):

                # Calculate lengths in range <100, min value depends on
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
                if lengthY > SpeedSettings.SPEED_FASTEST:
                    lengthY = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = overallSpeed
                speedRightWheel = int(lengthY)

                LOGGER.info(
                    "Steer right. Left wheel at speed: " + str(speedLeftWheel)
                    + " Right wheel at speed: " + str(speedRightWheel))

                robotmove.turn_forward(speedLeftWheel, speedRightWheel)
                time.sleep(STICK_DELAY)

            # Turn reverse left if joystick pushed bottom left outside central
            # channels
            elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                  and NunchukStickY < (NUNCHUK_MID - NUNCHUK_BUFFER)):

                # Calculate lengths in range <100, min value depends on
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
                if lengthY > SpeedSettings.SPEED_FASTEST:
                    lengthY = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = int(lengthY)
                speedRightWheel = overallSpeed

                LOGGER.info(
                    "Reverse left. Left wheel at speed: " + str(speedLeftWheel)
                    + " Right wheel at speed: " + str(speedRightWheel))

                robotmove.turn_reverse(speedLeftWheel, speedRightWheel)
                time.sleep(STICK_DELAY)

            # Turn reverse right if joystick pushed top right outside central
            # channels
            elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                  and NunchukStickY < (NUNCHUK_MID + NUNCHUK_BUFFER)):

                # Calculate lengths in range <100, min value depends on
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
                if lengthY > SpeedSettings.SPEED_FASTEST:
                    lengthY = SpeedSettings.SPEED_FASTEST

                # Calculate wheel speeds
                speedLeftWheel = overallSpeed
                speedRightWheel = int(lengthY)

                LOGGER.info("Reverse right. Left wheel at speed: " +
                            str(speedLeftWheel) + " Right wheel at speed: " +
                            str(speedRightWheel))

                robotmove.turn_reverse(speedLeftWheel, speedRightWheel)
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

            # If button A pressed centre Servo
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

            # If button C pressed Toggle the targeting laser
            if (buttons & cwiid.BTN_C):
                t = threading.Thread(
                    target=ToggleLaser, args=(
                        LOGGER,
                    ))
                t.start()
                time.sleep(BUTTON_DELAY)

            # If button z pressed Toggle the nerf gun fly wheels 
            if (buttons & cwiid.BTN_Z):
                t = threading.Thread(
                    target=ToggleMotor, args=(
                        LOGGER,
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
