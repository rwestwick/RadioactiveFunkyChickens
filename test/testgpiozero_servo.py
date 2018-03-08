from __future__ import division
import GPIOLayout
#from gpiozero import Servo
from gpiozero import AngularServo
from time import sleep

#servo = AngularServo(27, min_angle=-90, max_angle=90, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000, frame_width=5/1000)
servo = AngularServo(27, min_angle=-90, max_angle=90)
#servo = AngularServo(27)
print "setting to 0"
#servo.angle = 0
#print servo.value
sleep(2)
print "mid"
servo.mid()
#print servo.value
sleep(2)
print "min"
servo.min()
print servo.value
sleep(2)
print "mid"
servo.mid()
print servo.value
sleep(2)
print "max"
servo.max()
print servo.value
sleep(2)
print "-90"
servo.angle = -90
print servo.value
sleep(2)
print "90"
servo.angle = 90
print servo.value
sleep(2)
print "0"
servo.angle = 0
print servo.value
sleep(2)
servo.detach()
sleep(2)

exit()

servo1 = Servo(27, min_pulse_width=0.5 / 1000, max_pulse_width=2.5 / 1000, frame_width=20 / 1000)
servo1.min()
print servo.value
sleep(2)
servo1.mid()
print servo.value
sleep(2)
servo1.max()
print servo.value
sleep(2)
servo1.detach()

exit()
