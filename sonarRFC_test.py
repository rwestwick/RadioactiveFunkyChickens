import sonarRFC
import time

viewLeft = sonarRFC.leftUltraSensor()
viewRight = sonarRFC.rightUltraSensor()
time.sleep(1)

for i in range(20):
    leftDistance = viewLeft.Measurement()
    rightDistance = viewRight.Measurement()
    print("Average left distance: " + str(leftDistance) + " cm")
    time.sleep(1)
    print("Average right distance: " + str(rightDistance) + " cm")
    time.sleep(1)



