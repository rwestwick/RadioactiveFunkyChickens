#!/usr/bin/python

"""
Tests the algorithms for detecting where the black line, on a white background,
is in relation to the piCamera's frame.
"""

# Run this program from the command line as it allows the window to close when 'q' is pressed

# https://www.python.org/dev/peps/pep-0238/
# The future division statement, spelled "from __future__ import division", will change the / operator to mean true division
# throughout the module.
# This is needed for the row and column calculations for rectangle arrays to prevent rounding down to zero.
from __future__ import division

# Import needed libraries such as picamera OpenCV and NumPy
import cv2
import numpy as np
import time
from picamera import PiCamera
from picamera.array import PiRGBArray

# Initialize the camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
camera = PiCamera() # Initialize camera
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT) # resolution defaults to dosplays resolution
# Can get framerates up to 60fps 640x480
camera.framerate = 10 # If not set then defaults to 30fps
camera.vflip = True

# http://picamera.readthedocs.io/en/release-1.10/api_array.html
# class picamera.array.PiRGBArray(camera, size=None)[source]
# Produces a 3-dimensional RGB array from an RGB capture with the dimensions (rows, columns, plane)
# for example of size (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Allow the camera time to warmup
time.sleep(0.1)

# Initialize rowValues array to do testing such that they are all initialised to be white (255)
ROW_LENGTH = 10 # Number of rectangles for black/white analysis
rowMeanValues = np.ones(ROW_LENGTH) * 255

# Capture frames from the camera
# http://picamera.readthedocs.io/en/release-1.10/api_camera.html
# capture_continuous(output, format=None, use_video_port=False, resize=None, splitter_port=0, burst=False, **options)
# The format, use_video_port, splitter_port, resize, and options parameters are the same as in capture()

for frame in camera.capture_continuous(rawCapture, format="rgb", use_video_port=True):
    # grab the raw NumPy array respresenting the image, then intialize the timestap
    # and occupied/unoccupied text
    image = frame.array

    # Convert to gray scale
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Simple thresholding of gray sclae image
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

    # Create a frame for lower middle part of video
    # Top left is [0, 0] in [rows, columns]

    startRow = int(0.90 * CAMERA_HEIGHT)
    stopRow  = int(1.00 * CAMERA_HEIGHT) - 1.0
    startCol = int(0.40 * CAMERA_WIDTH)
    stopCol  = int(0.60 * CAMERA_WIDTH) - 1.0
    
    lowerMiddleSquare = threshImg[startRow:stopRow, startCol:stopCol]

    # show the frame    
    cv2.imshow("GrayFrame", gray)
    cv2.imshow("ThresholdFrame", threshImg)
    cv2.imshow("Frame", lowerMiddleSquare)

    # Capture a key press. The function waits argument in ms for any keyboard event
    key = cv2.waitKey(1) & 0xFF

    # Capture number of white/black pixels in ROW_LENGTH rectanges along lower row of threshold frame
    # N.B. May want to make this for several rows to track the line further in the horizon and
    # allow for sharp 90deg turns.
    
    for i in range(ROW_LENGTH):
        
        # Image region of interest (ROI)
        startRow = int(0.90 * CAMERA_HEIGHT)
        stopRow  = int(1.00 * CAMERA_HEIGHT) - 1.0
        startCol = int((i/ROW_LENGTH) * CAMERA_WIDTH)
        stopCol  = int(((i+1)/ROW_LENGTH) * CAMERA_WIDTH) - 1.0
        
        square = threshImg[startRow:stopRow, startCol:stopCol]

        # Mean of all the values in rectangular "square" array
        rowMeanValues[i] = int(np.mean(square))

        # Find index of first minimum mean value N.B. Black = 0, White = 255
        # As it is the first then if there are two fully black rectangles this could lead to errors
        smallSquare = np.argmin(rowMeanValues)
        print("The rectangle with the most black pixels is: ", str(smallSquare))

    # http://picamera.readthedocs.io/en/release-1.10/api_array.html
    # Clear the stream in preperation for the next frame
    rawCapture.truncate(0)

    # if the 'q' key was pressed break from the loop
    if key == ord("q"):
        break

# simply destroys all windows created
# Can use cv2.destroyWindow(frameName) to destroy a specific window
cv2.destroyAllWindows()
