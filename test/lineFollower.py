#!/usr/bin/python
# lineFollower.py

# Test PiWars python code

import robohat
import time

speed = 40
speedSpin = 80
durationSpin = 0.1

robohat.init()

# robohat.spinRight(90)
# time.sleep(2)
# robohat.spinLeft(90)
# time.sleep(2)


try:
    lastLineL = robohat.irLeftLine()
    lastLineR = robohat.irRightLine()
    # print 'LeftLine, RightLine:', lastLineL, lastLineR
    # print

    while True:
        newLineL = robohat.irLeftLine()
        newLineR = robohat.irRightLine()

        if (newLineL == False) and (newLineR == False):
            robohat.forward(speed)
            print 'Forward'

        elif (newLineL) and (newLineR == False):
            robohat.spinRight(speedSpin)
            print 'Spin Right'
            time.sleep(durationSpin)

        elif (newLineL == False) and (newLineR == True):
            robohat.spinLeft(speedSpin)
            print 'Spin Left'
            time.sleep(durationSpin)

        elif (newLineL) and (newLineR):
            robohat.stop()
            print 'Do not Panic'
            # break

            # if (newLineL != lastLineL) or (newLineR != lastLineR):
            #    print 'LeftLine, RightLine:', newLineL, newLineR
            #    print
            #    lastLineL = newLineL
            #    lastLineR = newLineR

        time.sleep(0.1)

except KeyboardInterrupt:
    print

finally:
    robohat.cleanup()
