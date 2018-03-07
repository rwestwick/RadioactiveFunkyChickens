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
import SetupConsoleLogger
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
    BUTTON_DELAY = 0.1  # seconds
    PAN_STEP = 10
    TILT_STEP = 10
    NUNCHUK_MAX = 220.0
    NUNCHUK_MID = 126.0
    NUNCHUK_MIN = 32.0
    NUNCHUK_BUFFER = 25.0

    def __init__(self, callback_b, callback_c, callback_z, 
                 servo_controller = None):
        """
        Initialise the parameters required for WiimoteNunchukControllerThread
        """
        MODULE_LOGGER.info("Setting up WiimoteNunchukControllerThread Module")

        self._mutex = threading.Lock()
        self._exit_now = False
        self._button_b_callback = callback_b
        self._button_c_callback = callback_c
        self._button_z_callback = callback_z
        self.speed = self.speedSettings.self.speed_FASTEST  # Initial forward speed
        self.t_val = 0  # 0 degrees is centre
        self.p_val = 0  # 0 degrees is centre

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
        if servo_controller is not None
            MODULE_LOGGER.debug("Using supplied servo controller")
            self.servo_controller = servo_controller
            self.servo_supplied = True
        else:
            MODULE_LOGGER.debug("Creating servo controller")
            self.servo_controller = ServoController.ServoController()
            self.servo_supplied = False
        
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

        if not self.servo_supplied:
            self.servo_controller.stop_servos()
            
        self.robotmove.cleanup()
        MODULE_LOGGER.info("Wiimote Controller Finished")

    def __wiimote_connection(self):
        """
        Performs the loop within the thread
        """
        self.wm = None
        i = 2
        while not self.wm:
            try:
                self.wm = cwiid.Wiimote()
            except RuntimeError:
                if (i > 5):
                    MODULE_LOGGER.error("Cannot create Wiimote connection.")
                    self.exit_now()
                MODULE_LOGGER.warning("Error opening wiimote connection, attempt " + str(i))
                i += 1
        
        if not self._exit_now
            MODULE_LOGGER.info("Wiimote connected.")
        
            # Set wiimote to report button presses
            self.wm.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_EXT

            # Turn on led to show connected
            self.wm.led = 1

    def exit_now(self):
        """
        Request the thread to exit
        """
        MODULE_LOGGER.debug("Request to exit")
        self._exit_now = True    

    def run(self):
        """
        Performs the loop within the thread
        """
        self.__wiimote_connection()
        
        MODULE_LOGGER.debug("Starting loop thread")
        while not self._exit_now:
            if 'nunchuk' in wm.state:

                buttons = wm.state['buttons']
                nunchuck_buttons = wmstate['nunchuk']['buttons']

                # X axis: Left Max = 25, Middle = 124, Right Max = 225
                NunchukStickX = (wm.state['nunchuk']['stick'][cwiid.X])
                # Y axis: Down Max = 30, Middle = 132, Up Max = 225
                NunchukStickY = (wm.state['nunchuk']['stick'][cwiid.Y])

                # Go forward if joystick pushed forward beyond buffer in central
                # channel
                if (NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)
                        and NunchukStickX < (NUNCHUK_MID + NUNCHUK_BUFFER)
                        and NunchukStickX > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                    self.speed = int(self.speedSettings.self.speed_FASTEST *
                                (NunchukStickY - NUNCHUK_MID) /
                                (NUNCHUK_MAX - NUNCHUK_MID))
                    if self.speed > self.speedSettings.self.speed_FASTEST:
                        self.speed = self.speedSettings.self.speed_FASTEST

                    MODULE_LOGGER.debug("Forward at self.speed " + str(self.speed))
                    robotmove.forward(self.speed)
                    time.sleep(STICK_DELAY)

                # Go backwards if joystick pulled back beyond buffer in central
                # channel
                elif (NunchukStickY < (NUNCHUK_MID - NUNCHUK_BUFFER)
                      and NunchukStickX < (NUNCHUK_MID + NUNCHUK_BUFFER)
                      and NunchukStickX > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                    self.speed = int(self.speedSettings.self.speed_FASTEST *
                                (NUNCHUK_MID - NunchukStickY) /
                                (NUNCHUK_MID - NUNCHUK_MIN))
                    if self.speed > self.speedSettings.self.speed_FASTEST:
                        self.speed = self.speedSettings.self.speed_FASTEST

                    MODULE_LOGGER.debug("Reverse at self.speed " + str(self.speed))
                    robotmove.reverse(self.speed)
                    time.sleep(STICK_DELAY)

                # Spin right right joystick pushed right beyond buffer in central
                # channel
                elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                      and NunchukStickY < (NUNCHUK_MID + NUNCHUK_BUFFER)
                      and NunchukStickY > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                    self.speed = int(self.speedSettings.self.speed_FASTEST *
                                (NunchukStickX - NUNCHUK_MID) /
                                (NUNCHUK_MAX - NUNCHUK_MID))
                    if self.speed > self.speedSettings.self.speed_FASTEST:
                        self.speed = self.speedSettings.self.speed_FASTEST

                    MODULE_LOGGER.debug("Spin right at self.speed " + str(self.speed))
                    robotmove.spin_right(self.speed)
                    time.sleep(STICK_DELAY)

                # Spin left if joystick pushed left beyond buffer in central
                # channel
                elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                      and NunchukStickY < (NUNCHUK_MID + NUNCHUK_BUFFER)
                      and NunchukStickY > (NUNCHUK_MID - NUNCHUK_BUFFER)):

                    self.speed = int(self.speedSettings.self.speed_FASTEST *
                                (NUNCHUK_MID - NunchukStickX) /
                                (NUNCHUK_MID - NUNCHUK_MIN))
                    if self.speed > self.speedSettings.self.speed_FASTEST:
                        self.speed = self.speedSettings.self.speed_FASTEST

                    MODULE_LOGGER.debug("Spin left at self.speed " + str(self.speed))
                    robotmove.spin_left(self.speed)
                    time.sleep(STICK_DELAY)

                # Turn forward left if joystick pushed top left outside central
                # channels
                elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                      and NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # NUNCHUK_BUFFER value
                    lengthX = self.speedSettings.self.speed_FASTEST * \
                        (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                    lengthY = self.speedSettings.self.speed_FASTEST * \
                        (NunchukStickY - NUNCHUK_MID) / (NUNCHUK_MAX - NUNCHUK_MID)

                    # self.speed is length of hypotenuse from Pythagoras
                    overallself.speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overallself.speed > self.speedSettings.self.speed_FASTEST:
                        overallself.speed = self.speedSettings.self.speed_FASTEST
                    if lengthY > self.speedSettings.self.speed_FASTEST:
                        lengthY = self.speedSettings.self.speed_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = int(lengthY)
                    self.speedRightWheel = overallself.speed

                    MODULE_LOGGER.debug(
                        "Steer left. Left wheel at self.speed: " + str(self.speedLeftWheel) +
                        " Right wheel at self.speed: " + str(self.speedRightWheel))
                    robotmove.turn_forward(self.speedLeftWheel, self.speedRightWheel)
                    time.sleep(STICK_DELAY)

                # Turn forward right if joystick pushed top right outside central
                # channels
                elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                      and NunchukStickY > (NUNCHUK_MID + NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # NUNCHUK_BUFFER value
                    lengthX = self.speedSettings.self.speed_FASTEST * \
                        (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                    lengthY = self.speedSettings.self.speed_FASTEST * \
                        (NunchukStickY - NUNCHUK_MID) / (NUNCHUK_MAX - NUNCHUK_MID)

                    # self.speed is length of hypotenuse from Pythagoras
                    overallself.speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overallself.speed > self.speedSettings.self.speed_FASTEST:
                        overallself.speed = self.speedSettings.self.speed_FASTEST
                    if lengthY > self.speedSettings.self.speed_FASTEST:
                        lengthY = self.speedSettings.self.speed_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = overallself.speed
                    self.speedRightWheel = int(lengthY)

                    MODULE_LOGGER.debug(
                        "Steer right. Left wheel at self.speed: " + str(self.speedLeftWheel)
                        + " Right wheel at self.speed: " + str(self.speedRightWheel))
                    robotmove.turn_forward(self.speedLeftWheel, self.speedRightWheel)
                    time.sleep(STICK_DELAY)

                # Turn reverse left if joystick pushed bottom left outside central
                # channels
                elif (NunchukStickX < (NUNCHUK_MID - NUNCHUK_BUFFER)
                      and NunchukStickY < (NUNCHUK_MID - NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # NUNCHUK_BUFFER value
                    lengthX = self.speedSettings.self.speed_FASTEST * \
                        (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                    lengthY = self.speedSettings.self.speed_FASTEST * \
                        (NUNCHUK_MID - NunchukStickY) / (NUNCHUK_MID - NUNCHUK_MIN)

                    # self.speed is length of hypotenuse from Pythagoras
                    overallself.speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overallself.speed > self.speedSettings.self.speed_FASTEST:
                        overallself.speed = self.speedSettings.self.speed_FASTEST
                    if lengthY > self.speedSettings.self.speed_FASTEST:
                        lengthY = self.speedSettings.self.speed_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = int(lengthY)
                    self.speedRightWheel = overallself.speed
                    
                    MODULE_LOGGER.debug(
                        "Reverse left. Left wheel at self.speed: " + str(self.speedLeftWheel)
                        + " Right wheel at self.speed: " + str(self.speedRightWheel))
                    robotmove.turn_reverse(self.speedLeftWheel, self.speedRightWheel)
                    time.sleep(STICK_DELAY)

                # Turn reverse right if joystick pushed top right outside central
                # channels
                elif (NunchukStickX > (NUNCHUK_MID + NUNCHUK_BUFFER)
                      and NunchukStickY < (NUNCHUK_MID + NUNCHUK_BUFFER)):

                    # Calculate lengths in range <100, min value depends on
                    # NUNCHUK_BUFFER value
                    lengthX = self.speedSettings.self.speed_FASTEST * \
                        (NUNCHUK_MID - NunchukStickX) / (NUNCHUK_MID - NUNCHUK_MIN)
                    lengthY = self.speedSettings.self.speed_FASTEST * \
                        (NUNCHUK_MID - NunchukStickY) / (NUNCHUK_MID - NUNCHUK_MIN)

                    # self.speed is length of hypotenuse from Pythagoras
                    overallself.speed = int(
                        math.sqrt(math.pow(lengthX, 2) + math.pow(lengthY, 2)))

                    if overallself.speed > self.speedSettings.self.speed_FASTEST:
                        overallself.speed = self.speedSettings.self.speed_FASTEST
                    if lengthY > self.speedSettings.self.speed_FASTEST:
                        lengthY = self.speedSettings.self.speed_FASTEST

                    # Calculate wheel self.speeds
                    self.speedLeftWheel = overallself.speed
                    self.speedRightWheel = int(lengthY)

                    MODULE_LOGGER.debug("Reverse right. Left wheel at self.speed: " +
                                str(self.speedLeftWheel) + " Right wheel at self.speed: " +
                                str(self.speedRightWheel))
                    robotmove.turn_reverse(self.speedLeftWheel, self.speedRightWheel)
                    time.sleep(STICK_DELAY)

                # else stop
                else:
                    robotmove.stop()

                    # If button up pressed move servo up
                if (buttons & cwiid.BTN_UP):
                    self.p_val = self.p_val + PAN_STEP
                    if self.p_val > 90:
                        self.p_val = 90
                    MODULE_LOGGER.debug("Servo Up to: " + str(self.p_val))
                    SERVO_CONTROLLER.set_pan_servo(self.p_val)
                    time.sleep(BUTTON_DELAY)

                # If button down pressed move servo down
                elif (buttons & cwiid.BTN_DOWN):
                    self.p_val = self.p_val - PAN_STEP
                    if self.p_val < -90:
                        self.p_val = -90
                    MODULE_LOGGER.debug("Servo Down to: " + str(self.p_val))
                    SERVO_CONTROLLER.set_pan_servo(self.p_val)
                    time.sleep(BUTTON_DELAY)

                # If button right pressed move servo right
                elif (buttons & cwiid.BTN_RIGHT):
                    self.t_val = self.t_val + TILT_STEP
                    if self.t_val > 90:
                        self.t_val = 90
                    MODULE_LOGGER.debug("Servo Right to: " + str(self.t_val))
                    SERVO_CONTROLLER.set_tilt_servo(self.t_val)
                    time.sleep(BUTTON_DELAY)

                # If button left pressed move servo left
                elif (buttons & cwiid.BTN_LEFT):
                    self.t_val = self.t_val - TILT_STEP
                    if self.t_val < -90:
                        self.t_val = -90
                    MODULE_LOGGER.debug("Servo Left to: " + str(self.t_val))
                    SERVO_CONTROLLER.set_tilt_servo(self.t_val)
                    time.sleep(BUTTON_DELAY)

                # If button A pressed centre Servo
                elif (buttons & cwiid.BTN_A):
                    self.p_val = 0
                    self.t_val = 0
                    SERVO_CONTROLLER.set_pan_servo(self.p_val)
                    SERVO_CONTROLLER.set_tilt_servo(self.t_val)
                    MODULE_LOGGER.debug("Centre!")
                    time.sleep(BUTTON_DELAY)

                # If button B pressed rumble Wiimote, but don't block other actions
                if (buttons & cwiid.BTN_B):
                    if self._button_b_callback is not None:              
                        t = threading.Thread(
                            target=self._button_b_callback, 
                            args=(wm, ))
                        t.start()
                        time.sleep(BUTTON_DELAY)

                # If button C pressed Toggle the targeting laser
                if (nunchuck_buttons & cwiid.NUNCHUK_BTN_C):
                    if self._button_c_callback is not None:              
                        t = threading.Thread(
                            target=self._button_c_callback, 
                            args=(wm, ))
                        t.start()
                        time.sleep(BUTTON_DELAY)

                # If button z pressed Toggle the nerf gun fly wheels
                if (nunchuck_buttons & cwiid.NUNCHUK_BTN_Z):
                    if self._button_z_callback is not None:              
                        t = threading.Thread(
                            target=self._button_z_callback, 
                            args=(wm, ))
                        t.start()
                        time.sleep(BUTTON_DELAY)

        MODULE_LOGGER.debug("Finished thread")
