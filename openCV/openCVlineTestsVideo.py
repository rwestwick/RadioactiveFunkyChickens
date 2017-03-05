# Run from command line allows the window to close when 'q' is pressed

# https://www.python.org/dev/peps/pep-0238/
# The future division statement, spelled "from __future__ import division", will change the / operator to mean true division
# throughout the module.
from __future__ import division

# Import OpenCV and NumPy
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Initialize the camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
ROW_LENGTH = 10.0
camera = PiCamera()
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
camera.framerate = 10
camera.vflip = True

# http://picamera.readthedocs.io/en/release-1.10/api_array.html
# class picamera.array.PiRGBArray(camera, size=None)[source]
# Produces a 3-dimensional RGB array from an RGB capture.
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Allow the camera time to warmup
time.sleep(0.1)

# Initialize rowValues array to do testing such that they are all initialised to be white
rowValues = np.ones(ROW_LENGTH) * 255

# capture frames from the camera
# http://picamera.readthedocs.io/en/release-1.10/api_camera.html
# capture_continuous(output, format=None, use_video_port=False, resize=None, splitter_port=0, burst=False, **options)
# The format, use_video_port, splitter_port, resize, and options parameters are the same as in capture()
for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    # grab the raw NumPy array respresenting the image, then intialize the timestap
    # and occupied/unoccupied text
    image = frame.array

    # Convert to gray scale
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Simple thresholding
    # http://www.pyimagesearch.com/2014/09/08/thresholding-simple-image-segmentation-using-opencv/
    # (T, threshImage) = cv2.threshold(src, thresh, maxval, type)
    # src - source image. This image should be grayscale.
    # thresh - is the threshold value which is used to classify the pixel intensities in the grayscale image.
    # maxval - is the pixel value used if any given pixel in the image passes the thresh  test.
    # type - the thresholding method to be used. The type  value can be any of:
    # cv2.THRESH_BINARY
    # cv2.THRESH_BINARY_INV
    # cv2.THRESH_TRUNC
    # cv2.THRESH_TOZERO
    # cv2.THRESH_TOZERO_INV
    ret,threshImg = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

##    print("Size of threshImg array: ", threshImg.shape)
##
##    startX = int((5.0/ROW_LENGTH) * CAMERA_WIDTH)
##    stopX = int(((5.0+1.0)/ROW_LENGTH) * CAMERA_WIDTH) - 1.0
##    startY = 0
##    stopY = int((1.0/ROW_LENGTH) * CAMERA_HEIGHT) - 1.0
##    square = threshImg[startX:stopX, startY:stopY]
##
##    print("Size of square array: ", square.shape)

    # show the frame    
    # cv2.imshow("Frame", gray)
    cv2.imshow("Frame", threshImg)
    # cv2.imshow("Frame", square)
    key = cv2.waitKey(1) & 0xFF

    for i in range(int(ROW_LENGTH)):
        # Image region of interest (ROI)
        startX = int((i/ROW_LENGTH) * CAMERA_WIDTH)
        stopX = int((((i+1.0)/ROW_LENGTH)  * CAMERA_WIDTH) - 1)
        startY = 0
        stopY = int(((1.0/ROW_LENGTH) * CAMERA_HEIGHT) - 1)
        
        square = threshImg[startX:stopX, startY:stopY]

        # print(square)

        # Sum and mean of all the values in square array
        rowValues[i] = int(np.mean(square))

        # Find indices of minimum value
        smallSquare = np.argmin(rowValues)
        print rowValues
        print smallSquare

    # http://picamera.readthedocs.io/en/release-1.10/api_array.html
    # Clear the stream in preperation for the next frame
    rawCapture.truncate(0)

    # if the 'q' key was pressed break from the loop
    if key == ord("q"):
        break

cv2.destroyAllWindows()
