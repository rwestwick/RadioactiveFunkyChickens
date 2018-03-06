import GPIOLayout
from gpiozero import AngularServo
from time import sleep

servo = AngularServo(GPIOLayout.DUCK_SHOOT_FIRE_GPIO)

servo.min()
sleep(1)
servo.mid()
sleep(1)
servo.max()
sleep(1)
servo.angle(-45)
sleep(1)
servo.angle(45)
sleep(1)
servo.detach()
sleep(1)
