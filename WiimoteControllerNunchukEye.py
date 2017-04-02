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

STICK_DELAY = 0.1
BUTTON_DELAY = 0.1

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
robotmove = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

# Initialise ServoController
SERVO_CONTROLLER = ServoController.ServoController()
SERVO_CONTROLLER.start_servos()

tVal = 0  # 0 degrees is centre
pVal = 0  # 0 degrees is centre

    
def main():
    """
    """
    # Set initial values
    speed = MotorController.SPEED_FASTEST  # Initial forward speed
    
    # Connecting to the wiimote. This allows several attempts
    # as first few often fail.
    LOGGER.info("Press 1+2 on your Wiimote now ...")
    time.sleep(1)

    wm = None
    i=2
    while not wm:
            try:
                    wm=cwiid.Wiimote()
            except RuntimeError:
                    if (i>5):
                            LOGGER.info("Cannot create Wiimote connection.")
                            quit()
                    LOGGER.info("Error opening wiimote connection, attempt "
                                + str(i))
                    i +=1

    LOGGER.info("Wiimote connected.")
    
    # Set wiimote to report button presses
    wm.rpt_mode = cwiid.RPT_BTN | cwiid.RPT_ACC | cwiid.RPT_EXT

    # Turn on led to show connected
    wm.led = 1

    # Respond to Nunchuk joystick
    while True:
        
        if wm.state.has_key('nunchuk'):
            # print("Success")

            buttons = wm.state['buttons']
            
            # X axis: Left Max = 25, Middle = 125, RightMax = 225
            NunchukStickX = (wm.state['nunchuk']['stick'][cwiid.X])
            # Y axis: DownMax = 30, Middle = 125, UpMax = 225
            NunchukStickY = (wm.state['nunchuk']['stick'][cwiid.Y])

            # print NunchukStickX
            # print NunchukStickY

            # Go forward if joystick pushed forward
            if (NunchukStickY > 150) & (NunchukStickY < 190):
                speed = MotorController.SPEED_SLOW
                robotmove.forward(speed)
                LOGGER.info("Forward at speed " + str(speed))
                time.sleep(STICK_DELAY)

            elif (NunchukStickY >= 190):
                speed = MotorController.SPEED_FASTEST
                robotmove.forward(speed)
                LOGGER.info("Forward at speed " + str(speed))
                time.sleep(STICK_DELAY)
            
            # Go backwards if joystick pulled back
            elif (NunchukStickY < 100) & (NunchukStickY > 50):
                speed = MotorController.SPEED_SLOW
                robotmove.reverse(speed)
                LOGGER.info("Reverse at speed " + str(speed))
                time.sleep(STICK_DELAY)
                
            elif (NunchukStickY <= 50):
                speed = MotorController.SPEED_FASTEST
                robotmove.reverse(speed)
                LOGGER.info("Reverse at speed " + str(speed))
                time.sleep(STICK_DELAY)

            # Spin right right joystick pushed right
            elif (NunchukStickX > 150) & (NunchukStickX < 190):
                speed = MotorController.SPEED_SLOW
                robotmove.spin_right(speed)
                LOGGER.info("Spin right at speed " + str(speed))
                time.sleep(STICK_DELAY)

            elif (NunchukStickX >= 190):
                speed = MotorController.SPEED_FASTEST
                robotmove.spin_right(speed)
                LOGGER.info("Spin right at speed " + str(speed))
                time.sleep(STICK_DELAY)
            
            # Spin left if joystick pushed left
            elif (NunchukStickX < 100) & (NunchukStickX > 50):
                speed = MotorController.SPEED_SLOW
                robotmove.spin_left(speed)
                LOGGER.info("Spin left at speed " + str(speed))
                time.sleep(STICK_DELAY)

            elif (NunchukStickX <= 50):
                speed = MotorController.SPEED_FASTEST
                robotmove.spin_left(speed)
                LOGGER.info("Spin left at speed " + str(speed))
                time.sleep(STICK_DELAY)

            # else stop
            else:
                robotmove.stop()
                LOGGER.info("Stop!")

            # Servo Controls
            
            # If button up pressed move servo up
            if (buttons & cwiid.BTN_UP):
                LOGGER.info("Servo Up")
                SERVO_CONTROLLER.set_pan_servo(90)
                time.sleep(BUTTON_DELAY)
                
            # If button down pressed move servo down
            elif (buttons & cwiid.BTN_DOWN):
                LOGGER.info("Servo Down")
                SERVO_CONTROLLER.set_pan_servo(-90)
                time.sleep(BUTTON_DELAY)

            # If button right pressed move servo right
            elif (buttons & cwiid.BTN_RIGHT):
                SERVO_CONTROLLER.set_tilt_servo(90)
                LOGGER.info("Servo Right")
                time.sleep(BUTTON_DELAY)
                
            # If button left pressed move servo left
            elif (buttons & cwiid.BTN_LEFT):
                LOGGER.info("Servo Left")
                SERVO_CONTROLLER.set_tilt_servo(-90)
                time.sleep(BUTTON_DELAY)

            # If botton A pressed centre Servo
            elif (buttons & cwiid.BTN_A):
                SERVO_CONTROLLER.set_pan_servo(0)
                SERVO_CONTROLLER.set_tilt_servo(0)
                LOGGER.info("Centre!") 
            
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
