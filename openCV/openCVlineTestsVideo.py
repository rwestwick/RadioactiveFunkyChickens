# Run from command line allows the window to 

# Import OpenCV and NumPy
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Initialize the camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
camera.vflip = True
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    # grab the raw NumPy array respresenting the image, then intialize the timestap
    # and occupied/unoccupied text
    image = frame.array

    # Convert to gray scale
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Simple thresholding
    ret,threshImg = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # show the frame    
    # cv2.imshow("Frame", gray)
    cv2.imshow("Frame", threshImg)
    key = cv2.waitKey(1) & 0xFF

    # clear the stream in preperation for the next frame
    rawCapture.truncate(0)

    # if the 'q' key was pressed break from the loop
    if key == ord("q"):
        break

cv2.destroyAllWindows()
