#!/usr/bin/python
"""
Provides an algorithm for following a black line on a white background.
"""

import logging
import MotorController
import LineFollowerSensor
import SetupConsoleLogger
import GPIOLayout
import KeyboardCharacterReader
import SpeedSettings

# Create a logger to both file and stdout
LOGGER = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(LOGGER)

# Initialise motors
ROBOTMOVE = DualMotorController.DualMotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)


def main():
    """
    Performs the main line following algorithm
    """
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
        GPIOLayout.LINE_FOLLOWER_LEFT_GPIO, 
        GPIOLayout.LINE_FOLLOWER_MIDDLE_GPIO,
        GPIOLayout.LINE_FOLLOWER_RIGHT_GPIO)

    while True:
        LSTATE = not linefollower.get_l_state()
        MSTATE = not linefollower.get_m_state()
        RSTATE = not linefollower.get_r_state()

        LOGGER.info(str(RSTATE) + " - " + str(MSTATE) + " - " + str(LSTATE))
        # time.sleep(1)

        if foundline is False:
            ROBOTMOVE.forward(SpeedSettings.SPEED_SLOW)
            if LSTATE == 1 or MSTATE == 1 or RSTATE == 1:
                foundline = True
        else:
            if LSTATE == 0 and MSTATE == 0 and RSTATE == 0:
                LOGGER.info("No Line")
                if lastturnleft:
                    LOGGER.info("  Hard Left Search")
                    ROBOTMOVE.spin_left(SpeedSettings.SPEED_VERYSLOW)
                elif lastturnright:
                    LOGGER.info("  Hard Right Search")
                    ROBOTMOVE.spin_right(SpeedSettings.SPEED_VERYSLOW)
                else:
                    LOGGER.info("  Search Right")
                    ROBOTMOVE.spin_right(SpeedSettings.SPEED_VERYSLOW)

            elif LSTATE == 0 and MSTATE == 1 and RSTATE == 0:
                LOGGER.info("Found Middle - Forward")
                ROBOTMOVE.forward(SpeedSettings.SPEED_VERYVERYSLOW)
                lastturnleft = False
                lastturnright = False
            elif LSTATE == 1 and MSTATE == 0 and RSTATE == 0:
                LOGGER.info("Left only - Turn Hard Left")
                ROBOTMOVE.spin_left(SpeedSettings.SPEED_SLOW)
                lastturnleft = True
                lastturnright = False
            elif LSTATE == 1 and MSTATE == 1 and RSTATE == 0:
                LOGGER.info("Left and middle - Turn Left")
                ROBOTMOVE.turn_forward(SpeedSettings.SPEED_SLOW / 2,
                                       SpeedSettings.SPEED_SLOW)
                lastturnleft = True
                lastturnright = False
            elif LSTATE == 0 and MSTATE == 0 and RSTATE == 1:
                LOGGER.info("Right only - Turn Hard Right")
                ROBOTMOVE.spin_right(SpeedSettings.SPEED_SLOW)
                lastturnleft = False
                lastturnright = True
            elif LSTATE == 0 and MSTATE == 1 and RSTATE == 1:
                LOGGER.info("Right and middle - Turn Right")
                ROBOTMOVE.turn_forward(SpeedSettings.SPEED_SLOW,
                                       SpeedSettings.SPEED_SLOW / 2)
                lastturnleft = False
                lastturnright = True
            elif LSTATE == 1 and MSTATE == 1 and RSTATE == 1:
                LOGGER.info("All sensors are on the line")
                ROBOTMOVE.forward(SpeedSettings.SPEED_MEDIUM)
            else:
                LOGGER.error("Some other state")
                ROBOTMOVE.reverse(SpeedSettings.SPEED_MEDIUM)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping the race")
    finally:
        LOGGER.info("Line Follower Finished")
        ROBOTMOVE.cleanup()
