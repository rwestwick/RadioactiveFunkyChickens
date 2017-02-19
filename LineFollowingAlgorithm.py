#!/usr/bin/python

"""
Provides an algorithm for following a black line on a white background.
"""

import logging
from . import MotorController
from . import LineFollowerSensor
from . import SetupConsoleLogger
from . import GPIOLayout
from . import KeyboardCharacterReader


# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
robotmove = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)


def main():
    LOGGER.info("Line Follower With Move")

    # Waiting for start of race
    LOGGER.info("To start line following press 'Space' key.")

    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    foundline = False
    lastturnleft = False
    lastturnright = False
    linefollower = LineFollowerSensor.LineFollowerSensor(
        GPIOLayout.LINE_FOLLOWER_LEFT_PIN,
        GPIOLayout.LINE_FOLLOWER_MIDDLE_PIN,
        GPIOLayout.LINE_FOLLOWER_RIGHT_PIN)

    while True:
        LSTATE = linefollower.get_l_state()
        MSTATE = linefollower.get_m_state()
        RSTATE = linefollower.get_r_state()

        if foundline == False:
            robotmove.forward(MotorController.SPEED_SLOW)
            if LSTATE == 1 or MSTATE == 1 or RSTATE == 1:
                foundline = True
        else:
            if LSTATE == 0 and MSTATE == 0 and RSTATE == 0:
                LOGGER.info("No Line")
                if lastturnleft:
                    LOGGER.info("  Hard Left Search")
                    robotmove.spin_left(MotorController.SPEED_VERYVERYSLOW)
                elif lastturnright:
                    LOGGER.info("  Hard Right Search")
                    robotmove.spin_right(MotorController.SPEED_VERYVERYSLOW)
                else:
                    LOGGER.info("  Search Right")
                    robotmove.spin_right(MotorController.SPEED_VERYVERYSLOW)

            elif LSTATE == 0 and MSTATE == 1 and RSTATE == 0:
                LOGGER.info("Found Middle")
                # if lastturnleft == True:
                #     logger.info("  Small Hard Right Adjustment")
                #     robotmove.SetSpeedVeryVerySlow()
                #     robotmove.TurnBackRight()
                # elif lastturnright == True:
                #     logger.info("  Small Hard Left Adjustment")
                #     robotmove.SetSpeedVeryVerySlow()
                #     robotmove.TurnBackLeft()
                LOGGER.info("  Forward")
                robotmove.forward(MotorController.SPEED_FAST)
                lastturnleft = False
                lastturnright = False
            elif LSTATE == 1 and MSTATE == 0 and RSTATE == 0:
                LOGGER.info("Left only - Turn Hard Left")
                robotmove.spin_left(MotorController.SPEED_SLOW)
                lastturnleft = True
                lastturnright = False
            elif LSTATE == 1 and MSTATE == 1 and RSTATE == 0:
                LOGGER.info("Left and middle - Turn Left")
                robotmove.turn_forward(MotorController.SPEED_SLOW / 2,
                                       MotorController.SPEED_SLOW)
                lastturnleft = True
                lastturnright = False
            elif LSTATE == 0 and MSTATE == 0 and RSTATE == 1:
                LOGGER.info("Right only - Turn Hard Right")
                robotmove.spin_right(MotorController.SPEED_SLOW)
                lastturnleft = False
                lastturnright = True
            elif LSTATE == 0 and MSTATE == 1 and RSTATE == 1:
                LOGGER.info("Right and middle - Turn Right")
                robotmove.turn_forward(MotorController.SPEED_SLOW,
                                       MotorController.SPEED_SLOW / 2)
                lastturnleft = False
                lastturnright = True
            elif LSTATE == 1 and MSTATE == 1 and RSTATE == 1:
                LOGGER.info("All sensors are on the line")
                robotmove.forward(MotorController.SPEED_MEDIUM)
            else:
                LOGGER.error("Some other state")
                robotmove.reverse(MotorController.SPEED_MEDIUM)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the race")
    finally:
        LOGGER.info("Line Follower Finished")
        robotmove.cleanup()
