#!/usr/bin/python

"""
Provides an algorithm for following a black line on a white background.
"""

import logging
import MotorController
import LineFollowerSensor
import SetupConsoleLogger
import GPIOLayout

logger = logging.getLogger(__name__)
SetupConsoleLogger.setup_console_logger(logger)

# Initialise motors
robotmove = MotorController.MotorController(GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
                                            GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
                                            GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
                                            GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)


def main():
    logger.info("Line Follower With Move")
    foundline = False
    lastturnleft = False
    lastturnright = False
    linefollower = LineFollowerSensor.LineFollowerSensor(
        GPIOLayout.LINE_FOLLOWER_LEFT,
        GPIOLayout.LINE_FOLLOWER_MIDDLE,
        GPIOLayout.LINE_FOLLOWER_RIGHT)

    while True:
        LSTATE = linefollower.get_l_state()
        MSTATE = linefollower.get_m_state()
        RSTATE = linefollower.get_r_state()

        if foundline == False:
            robotmove.forward(robotmove.SPEED_SLOW)
            if LSTATE == 1 or MSTATE == 1 or RSTATE == 1:
                foundline = True
        else:
            if LSTATE == 0 and MSTATE == 0 and RSTATE == 0:
                logger.info("No Line")
                if lastturnleft:
                    logger.info("  Hard Left Search")
                    robotmove.spin_left(robotmove.SPEED_VERYVERYSLOW)
                elif lastturnright:
                    logger.info("  Hard Right Search")
                    robotmove.spin_right(robotmove.SPEED_VERYVERYSLOW)
                else:
                    logger.info("  Search Right")
                    robotmove.spin_right(robotmove.SPEED_VERYVERYSLOW)

            elif LSTATE == 0 and MSTATE == 1 and RSTATE == 0:
                logger.info("Found Middle")
                """
                if lastturnleft == True:
                    logger.info("  Small Hard Right Adjustment")
                    robotmove.SetSpeedVeryVerySlow()
                    robotmove.TurnBackRight()
                elif lastturnright == True:
                    logger.info("  Small Hard Left Adjustment")
                    robotmove.SetSpeedVeryVerySlow()
                    robotmove.TurnBackLeft()
                """
                logger.info("  Forward")
                robotmove.forward(robotmove.SPEED_FAST)
                lastturnleft = False
                lastturnright = False
            elif (LSTATE == 1 and MSTATE == 0 and RSTATE == 0):
                logger.info("Left only - Turn Hard Left")
                robotmove.spin_left(robotmove.SPEED_SLOW)
                lastturnleft = True
                lastturnright = False
            elif (LSTATE == 1 and MSTATE == 1 and RSTATE == 0):
                logger.info("Left and middle - Turn Left")
                robotmove.one_wheel_left(robotmove.SPEED_SLOW)
                lastturnleft = True
                lastturnright = False
            elif (LSTATE == 0 and MSTATE == 0 and RSTATE == 1):
                logger.info("Right only - Turn Hard Right")
                robotmove.spin_right(robotmove.SPEED_SLOW)
                lastturnleft = False
                lastturnright = True
            elif (LSTATE == 0 and MSTATE == 1 and RSTATE == 1):
                logger.info("Right and middle - Turn Right")
                robotmove.one_wheel_right(robotmove.SPEED_SLOW)
                lastturnleft = False
                lastturnright = True
            elif LSTATE == 1 and MSTATE == 1 and RSTATE == 1:
                logger.info("All sensors are on the line")
                robotmove.forward(robotmove.SPEED_MEDIUM)
            else:
                logger.error("Some other state")
                robotmove.reverse(robotmove.SPEED_MEDIUM)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Line Follower Finished")
        robotmove.cleanup()
