#!/usr/bin/python
"""
Class defines how to interact with a Wiimote and Nunchuk
Controller via a thread

Need to install correct python modules, see
https://help.ubuntu.com/community/CWiiD
"""

import time
import logging
import threading
import math
import cwiid
import ServoController
import DualMotorController
import GPIOLayout
import SpeedSettings

MODULE_LOGGER = logging.getLogger("__main__.WiimoteNunchukControllerThread")


class WiimoteNunchukControllerThread(threading.Thread):
    """
    Defines the generic wiimote and nunchuk control mechanism
    """
    # Set constants
    STICK_DELAY = 0.1  # seconds
    BUTTON_DELAY = 0.25  # seconds
    PAN_STEP = 10
    TILT_STEP = 10
    NUNCHUK_MAX = 220.0
    NUNCHUK_MID = 126.0
    NUNCHUK_MIN = 32.0
    NUNCHUK_BUFFER = 25.0

    def __init__(self,
                 callback_b,
                 callback_c,
                 callback_z,
                 servo_controller=None):
        """
        Initialise the parameters required for WiimoteNunchukControllerThread
        """
        MODULE_LOGGER.info("Setting up WiimoteNunchukControllerThread Module")

        self._mutex = threading.Lock()
        self.t_val = 0  # 0 degrees is centre
        self.p_val = 0  # 0 degrees is centre

        self._exit_now = False
        self._servo_supplied = False
        self._button_b_callback = callback_b
        self._button_c_callback = callback_c
        self._button_z_callback = callback_z
        self.speed = SpeedSettings.SPEED_FASTEST  # Initial forward speed
        self.servo_controller = None

        # Initialise motors
        self.robotmove = DualMotorController.DualMotorController(
            GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
            GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)

        # Initialise ServoController
        if servo_controller is not None:
            MODULE_LOGGER.debug("Using supplied servo controller")
            self.servo_controller = servo_controller
            self._servo_supplied = True
        else:
            MODULE_LOGGER.debug("Creating servo controller")
            self.servo_controller = ServoController.ServoController()
            self._servo_supplied = False

        self.servo_controller.start_servos()

        # Wiimote Controller
        self.wm = None

        threading.Thread.__init__(self)

    def __del__(self):
        """
        Destructor
        """
        if not self._exit_now:
            self.exit_now()

        if not self._servo_supplied and self.servo_controller is not None:
            self.servo_controller.stop_servos()

        self.robotmove.cleanup()
        MODULE_LOGGER.info("Wiimote Controller Finished")

    def __wiimote_connection(self):
        """
        Performs the loop within the thread
        """
        MODULE_LOGGER.info("Press 1+2 on your Wiimote now ...")

        i = 1
        while not self.wm and not self._exit_now:
            try:
                self.wm = cwiid.Wiimote()
            except RuntimeError:
                if (i > 5):
                    MODULE_LOGGER.error("Cannot create Wiimote connection.")
                    self.exit_now()
                MODULE_LOGGER.warning(
                    "Error opening wiimote connection, attempt " + str(i))
                i += 1

        if not self._exit_now:
            MODULE_LOGGER.info("Wiimote connected.")

            # Set wiimote to report button presses
            self.wm.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_EXT

            # Turn on led to show connected
            self.wm.led = 1

    def exit_now(self):
        """
        Request the thread to exit
        """
        MODULE_LOGGER.info("Request to exit")
        self._exit_now = True

    def run(self):
        """
        Performs the loop within the thread
        """
        self.__wiimote_connection()

        MODULE_LOGGER.debug("Starting loop thread")
        while not self._exit_now:
            if 'nunchuk' in self.wm.state:

                buttons = self.wm.state['buttons']
                nunchuck_buttons = self.wm.state['nunchuk']['buttons']

                # X axis: Left Max = 25, Middle = 124, Right Max = 225
                NunchukStickX = (self.wm.state['nunchuk']['stick'][cwiid.X])
                # Y axis: Down Max = 30, Middle = 132, Up Max = 225
                NunchukStickY = (self.wm.state['nunchuk']['stick'][cwiid.Y])

                # Go forward if joystick pushed forward beyond buffer in central
                # channel
                if (NunchukStickY > (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                        and NunchukStickX <
                    (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                        and NunchukStickX >
                        (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)):

                    self.speed = int(SpeedSettings.SPEED_FASTEST *
                                     (NunchukStickY - self.NUNCHUK_MID) /
                                     (self.NUNCHUK_MAX - self.NUNCHUK_MID))
                    if self.speed > SpeedSettings.SPEED_FASTEST:
                        self.speed = SpeedSettings.SPEED_FASTEST

                    MODULE_LOGGER.debug("Forward at speed " + str(self.speed))
                    self.robotmove.forward(self.speed)
                    time.sleep(self.STICK_DELAY)

                # Go backwards if joystick pulled back beyond buffer in central
                # channel
                elif (NunchukStickY < (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)
                      and NunchukStickX <
                      (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                      and NunchukStickX >
                      (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)):

                    self.speed = int(SpeedSettings.SPEED_FASTEST *
                                     (self.NUNCHUK_MID - NunchukStickY) /
                                     (self.NUNCHUK_MID - self.NUNCHUK_MIN))
                    if self.speed > SpeedSettings.SPEED_FASTEST:
                        self.speed = SpeedSettings.SPEED_FASTEST

                    MODULE_LOGGER.debug("Reverse at speed " + str(self.speed))
                    self.robotmove.reverse(self.speed)
                    time.sleep(self.STICK_DELAY)

                # Spin right right joystick pushed right beyond buffer in central
                # channel
                elif (NunchukStickX > (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                      and NunchukStickY <
                      (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                      and NunchukStickY >
                      (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)):

                    self.speed = int(SpeedSettings.SPEED_FASTEST *
                                     (NunchukStickX - self.NUNCHUK_MID) /
                                     (self.NUNCHUK_MAX - self.NUNCHUK_MID))
                    if self.speed > SpeedSettings.SPEED_FASTEST:
                        self.speed = SpeedSettings.SPEED_FASTEST

                    MODULE_LOGGER.debug(
                        "Spin right at speed " + str(self.speed))
                    self.robotmove.spin_right(self.speed)
                    time.sleep(self.STICK_DELAY)

                # Spin left if joystick pushed left beyond buffer in central
                # channel
                elif (NunchukStickX < (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)
                      and NunchukStickY <
                      (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                      and NunchukStickY >
                      (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)):

                    self.speed = int(SpeedSettings.SPEED_FASTEST *
                                     (self.NUNCHUK_MID - NunchukStickX) /
                                     (self.NUNCHUK_MID - self.NUNCHUK_MIN))
                    if self.speed > SpeedSettings.SPEED_FASTEST:
                        self.speed = SpeedSettings.SPEED_FASTEST

                    MODULE_LOGGER.debug(
                        "Spin left at speed " + str(self.speed))
                    self.robotmove.spin_left(self.speed)
                    time.sleep(self.STICK_DELAY)

                # Turn forward left if joystick pushed top left outside central
                # channels
                elif (NunchukStickX < (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)
                      and NunchukStickY >
                      (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # self.NUNCHUK_BUFFER value
                    lengthX = SpeedSettings.SPEED_FASTEST * \
                        (self.NUNCHUK_MID - NunchukStickX) / (self.NUNCHUK_MID - self.NUNCHUK_MIN)
                    lengthY = SpeedSettings.SPEED_FASTEST * \
                        (NunchukStickY - self.NUNCHUK_MID) / (self.NUNCHUK_MAX - self.NUNCHUK_MID)

                    # self.speed is length of hypotenuse from Pythagoras
                    overall_speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overall_speed > SpeedSettings.SPEED_FASTEST:
                        overall_speed = SpeedSettings.SPEED_FASTEST
                    if lengthY > SpeedSettings.SPEED_FASTEST:
                        lengthY = SpeedSettings.SPEED_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = int(lengthY)
                    self.speedRightWheel = overall_speed

                    MODULE_LOGGER.debug(
                        "Steer left. Left wheel at speed: " +
                        str(self.speedLeftWheel) + " Right wheel at speed: " +
                        str(self.speedRightWheel))
                    self.robotmove.turn_forward(self.speedLeftWheel,
                                                self.speedRightWheel)
                    time.sleep(self.STICK_DELAY)

                # Turn forward right if joystick pushed top right outside central
                # channels
                elif (NunchukStickX > (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                      and NunchukStickY >
                      (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # self.NUNCHUK_BUFFER value
                    lengthX = SpeedSettings.SPEED_FASTEST * \
                        (self.NUNCHUK_MID - NunchukStickX) / (self.NUNCHUK_MID - self.NUNCHUK_MIN)
                    lengthY = SpeedSettings.SPEED_FASTEST * \
                        (NunchukStickY - self.NUNCHUK_MID) / (self.NUNCHUK_MAX - self.NUNCHUK_MID)

                    # self.speed is length of hypotenuse from Pythagoras
                    overall_speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overall_speed > SpeedSettings.SPEED_FASTEST:
                        overall_speed = SpeedSettings.SPEED_FASTEST
                    if lengthY > SpeedSettings.SPEED_FASTEST:
                        lengthY = SpeedSettings.SPEED_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = overall_speed
                    self.speedRightWheel = int(lengthY)

                    MODULE_LOGGER.debug(
                        "Steer right. Left wheel at speed: " +
                        str(self.speedLeftWheel) + " Right wheel at speed: " +
                        str(self.speedRightWheel))
                    self.robotmove.turn_forward(self.speedLeftWheel,
                                                self.speedRightWheel)
                    time.sleep(self.STICK_DELAY)

                # Turn reverse left if joystick pushed bottom left outside central
                # channels
                elif (NunchukStickX < (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)
                      and NunchukStickY <
                      (self.NUNCHUK_MID - self.NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # self.NUNCHUK_BUFFER value
                    lengthX = SpeedSettings.SPEED_FASTEST * \
                        (self.NUNCHUK_MID - NunchukStickX) / (self.NUNCHUK_MID - self.NUNCHUK_MIN)
                    lengthY = SpeedSettings.SPEED_FASTEST * \
                        (self.NUNCHUK_MID - NunchukStickY) / (self.NUNCHUK_MID - self.NUNCHUK_MIN)

                    # self.speed is length of hypotenuse from Pythagoras
                    overall_speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overall_speed > SpeedSettings.SPEED_FASTEST:
                        overall_speed = SpeedSettings.SPEED_FASTEST
                    if lengthY > SpeedSettings.SPEED_FASTEST:
                        lengthY = SpeedSettings.SPEED_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = int(lengthY)
                    self.speedRightWheel = overall_speed

                    MODULE_LOGGER.debug(
                        "Reverse left. Left wheel at speed: " +
                        str(self.speedLeftWheel) + " Right wheel at speed: " +
                        str(self.speedRightWheel))
                    self.robotmove.turn_reverse(self.speedLeftWheel,
                                                self.speedRightWheel)
                    time.sleep(self.STICK_DELAY)

                # Turn reverse right if joystick pushed top right outside central
                # channels
                elif (NunchukStickX > (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)
                      and NunchukStickY <
                      (self.NUNCHUK_MID + self.NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # self.NUNCHUK_BUFFER value
                    lengthX = SpeedSettings.SPEED_FASTEST * \
                        (self.NUNCHUK_MID - NunchukStickX) / (self.NUNCHUK_MID - self.NUNCHUK_MIN)
                    lengthY = SpeedSettings.SPEED_FASTEST * \
                        (self.NUNCHUK_MID - NunchukStickY) / (self.NUNCHUK_MID - self.NUNCHUK_MIN)

                    # self.speed is length of hypotenuse from Pythagoras
                    overall_speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overall_speed > SpeedSettings.SPEED_FASTEST:
                        overall_speed = SpeedSettings.SPEED_FASTEST
                    if lengthY > SpeedSettings.SPEED_FASTEST:
                        lengthY = SpeedSettings.SPEED_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = overall_speed
                    self.speedRightWheel = int(lengthY)

                    MODULE_LOGGER.debug(
                        "Reverse right. Left wheel at speed: " +
                        str(self.speedLeftWheel) + " Right wheel at speed: " +
                        str(self.speedRightWheel))
                    self.robotmove.turn_reverse(self.speedLeftWheel,
                                                self.speedRightWheel)
                    time.sleep(self.STICK_DELAY)

                # else stop
                else:
                    self.robotmove.stop()

                    # If button up pressed move servo up
                if (buttons & cwiid.BTN_UP):
                    self.p_val = self.p_val + self.PAN_STEP
                    if self.p_val > 90:
                        self.p_val = 90
                    MODULE_LOGGER.debug("Servo Up to: " + str(self.p_val))
                    self.servo_controller.set_pan_servo(self.p_val)
                    time.sleep(self.BUTTON_DELAY)

                # If button down pressed move servo down
                elif (buttons & cwiid.BTN_DOWN):
                    self.p_val = self.p_val - self.PAN_STEP
                    if self.p_val < -90:
                        self.p_val = -90
                    MODULE_LOGGER.debug("Servo Down to: " + str(self.p_val))
                    self.servo_controller.set_pan_servo(self.p_val)
                    time.sleep(self.BUTTON_DELAY)

                # If button right pressed move servo right
                elif (buttons & cwiid.BTN_RIGHT):
                    self.t_val = self.t_val + self.TILT_STEP
                    if self.t_val > 90:
                        self.t_val = 90
                    MODULE_LOGGER.debug("Servo Right to: " + str(self.t_val))
                    self.servo_controller.set_tilt_servo(self.t_val)
                    time.sleep(self.BUTTON_DELAY)

                # If button left pressed move servo left
                elif (buttons & cwiid.BTN_LEFT):
                    self.t_val = self.t_val - self.TILT_STEP
                    if self.t_val < -90:
                        self.t_val = -90
                    MODULE_LOGGER.debug("Servo Left to: " + str(self.t_val))
                    self.servo_controller.set_tilt_servo(self.t_val)
                    time.sleep(self.BUTTON_DELAY)

                # If button A pressed centre Servo
                elif (buttons & cwiid.BTN_A):
                    self.p_val = 0
                    self.t_val = 0
                    self.servo_controller.set_pan_servo(self.p_val)
                    self.servo_controller.set_tilt_servo(self.t_val)
                    MODULE_LOGGER.debug("Centre!")
                    time.sleep(self.BUTTON_DELAY)

                # If button B pressed rumble Wiimote, but don't block other
                # actions
                if (buttons & cwiid.BTN_B):
                    if self._button_b_callback is not None:
                        self._button_b_callback(self.wm)
                        # t = threading.Thread(
                        #    target=self._button_b_callback,
                        #    args=(self.wm, ))
                        # t.start()
                        time.sleep(self.BUTTON_DELAY)

                # If button C pressed Toggle the targeting laser
                if (nunchuck_buttons & cwiid.NUNCHUK_BTN_C):
                    if self._button_c_callback is not None:
                        self._button_c_callback(self.wm)
                        # t = threading.Thread(
                        #    target=self._button_c_callback,
                        #    args=(self.wm, ))
                        # t.start()
                        time.sleep(self.BUTTON_DELAY)

                # If button z pressed Toggle the nerf gun fly wheels
                if (nunchuck_buttons & cwiid.NUNCHUK_BTN_Z):
                    if self._button_z_callback is not None:
                        self._button_z_callback(self.wm)
                        # t = threading.Thread(
                        #    target=self._button_z_callback,
                        #    args=(self.wm, ))
                        # t.start()
                        time.sleep(self.BUTTON_DELAY)

        MODULE_LOGGER.debug("Finished thread")
