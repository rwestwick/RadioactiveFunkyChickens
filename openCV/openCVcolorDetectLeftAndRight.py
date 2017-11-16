#!/usr/bin/python
# openCVcolorDetectLeftAndRight.py

"""
Tests the algorithms for detecting where the coloured balls are on a black background
is in relation to the piCamera's frame.
This will be used for the "Somewhere Over Rainbow" Pi Wars challenge.
http://piwars.org/2018-competition/challenges/somewhere-over-the-rainbow/
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

# Define Colour constants

# Define the colour boundaries in BGR
LOWER_RED_BGR = [15, 15, 100] # Red
UPPER_RED_BGR = [100, 100, 255] # Red
LOWER_BLUE_BGR = [100, 15, 15] # Blue
UPPER_BLUE_BGR = [255, 100, 100] # Blue
LOWER_GREEN_BGR = [15, 100, 15] # Green
UPPER_GREEN_BGR = [100, 255, 100] # Green
LOWER_YELLOW_BGR = [15, 100, 100] # Yellow
UPPER_YELLOW_BGR = [100, 255, 255] # Yellow
LOWER_BGR_ARRAY = [LOWER_RED_BGR, LOWER_BLUE_BGR, LOWER_GREEN_BGR, LOWER_GREEN_BGR]
UPPER_BGR_ARRAY = [UPPER_RED_BGR, UPPER_BLUE_BGR, UPPER_GREEN_BGR, UPPER_GREEN_BGR]

# Define the colour boundaries in HSV
# http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
# For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255].
# Different softwares use different scales. So if you are comparing OpenCV values with them,
# you need to normalize these ranges.
# [255, 0, 0] BGR -> [120, 255, 255] HSV - Blue
# [0, 255, 0] BGR -> [60, 255, 255] HSV - Green
# [0, 0, 255] BGR -> [0, 255, 255] HSV - Red - For red may need two ranges
# https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
# https://pythonprogramming.net/color-filter-python-opencv-tutorial/
# [0, 255, 255] BGR -> [30, 255, 255] HSV - Yellow - Half Red and Green

LOWER_RED_HSV = [0, 50, 50] # Red
UPPER_RED_HSV = [20, 155, 255] # Red
LOWER_BLUE_HSV = [90, 50, 50] # Blue
UPPER_BLUE_HSV = [150, 255, 255] # Blue
LOWER_GREEN_HSV = [45, 50, 50] # Green
UPPER_GREEN_HSV = [90, 155, 255] # Green
LOWER_YELLOW_HSV = [20, 50, 50] # Yellow
UPPER_YELLOW_HSV = [45, 255, 255] # Yellow
LOWER_HSV_ARRAY = [LOWER_RED_HSV, LOWER_BLUE_HSV, LOWER_GREEN_HSV, LOWER_YELLOW_HSV]
UPPER_HSV_ARRAY = [UPPER_RED_HSV, UPPER_BLUE_HSV, UPPER_GREEN_HSV, UPPER_YELLOW_HSV]

# Initialize colour array counter
colourArrayCntr = 0
COLOUR_NAME_ARRAY = ['Red', 'Blue', 'Green', 'Yellow']

# Initialize the camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
camera = PiCamera() # Initialize camera
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT) # resolution defaults to dosplays resolution
# Can get framerates up to 60fps 640x480
camera.framerate = 10 # If not set then defaults to 30fps
camera.vflip = True
camera.hflip = True

# http://picamera.readthedocs.io/en/release-1.10/api_array.html
# class picamera.array.PiRGBArray(camera, size=None)[source]
# Produces a 3-dimensional RGB array from an RGB capture with the dimensions (rows, columns, plane)
# for example of size (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Allow the camera time to warmup
time.sleep(0.1)

# Initialize rowValues array to do testing such that they are all initialised to be white (255)
ROW_LENGTH = 2 # Number of rectangles per row for analysis
COL_LENGTH = 2 # Number of rectangles per column for analysis
MeanValuesBGR = np.ones([COL_LENGTH, ROW_LENGTH]) * 255
MeanValuesHSV = np.ones([COL_LENGTH, ROW_LENGTH]) * 255

# Capture frames from the camera
# http://picamera.readthedocs.io/en/release-1.10/api_camera.html
# capture_continuous(output, format=None, use_video_port=False, resize=None, splitter_port=0, burst=False, **options)
# The format, use_video_port, splitter_port, resize, and options parameters are the same as in capture()

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array respresenting the image, then intialize the timestap
    # and occupied/unoccupied text
    image = frame.array

    bgrImage = image # Keep in BGR
    hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # Convert BGR to HSV

    # Select colour range to detect
    lower_bgr = LOWER_BGR_ARRAY[colourArrayCntr]
    upper_bgr = UPPER_BGR_ARRAY[colourArrayCntr]

    lower_hsv = LOWER_HSV_ARRAY[colourArrayCntr]
    upper_hsv = UPPER_HSV_ARRAY[colourArrayCntr]

    # Create NumPy arrays from the boundaries
    lower_bgr = np.array(lower_bgr, dtype = "uint8")
    upper_bgr = np.array(upper_bgr, dtype = "uint8")

    lower_hsv = np.array(lower_hsv, dtype = "uint8")
    upper_hsv = np.array(upper_hsv, dtype = "uint8")

    # Find the colours within the specified boundaries and apply the mask - BGR
    mask_bgr = cv2.inRange(bgrImage, lower_bgr, upper_bgr)
    output_bgr = cv2.bitwise_and(bgrImage, bgrImage, mask = mask_bgr)

    # Find the colours within the specified boundaries and apply the mask - HSV
    mask_hsv = cv2.inRange(hsvImage, lower_hsv, upper_hsv)
    output_hsv = cv2.bitwise_and(hsvImage, hsvImage, mask = mask_hsv)

    # Create a frame for lower middle part of video
    # Top left is [0, 0] in [rows, columns]

    startRow = int(0.90 * CAMERA_HEIGHT)
    stopRow  = int(1.00 * CAMERA_HEIGHT) - 1.0
    startCol = int(0.40 * CAMERA_WIDTH)
    stopCol  = int(0.60 * CAMERA_WIDTH) - 1.0
    
    # Show the frame(s)
    cv2.imshow("BGR ColourFrame", bgrImage)
    cv2.imshow("BGR Mask", mask_bgr)
    cv2.imshow("BGR ColourThreshold", output_bgr)
    cv2.imshow("HSV Mask", mask_hsv)
    cv2.imshow("HSV ColourThreshold", output_hsv)

    # Capture mean number of white/black pixels in the mask output in each ROW and COLUMN block

    # Loop over all rows
    for j in range(COL_LENGTH):
        
        # Loop over all columns
        for i in range(ROW_LENGTH):
        
            # Image region of interest (ROI)
            startRow = int((j/COL_LENGTH) * CAMERA_HEIGHT)
            stopRow  = int(((j+1)/COL_LENGTH) * CAMERA_HEIGHT) - 1.0
            startCol = int((i/ROW_LENGTH) * CAMERA_WIDTH)
            stopCol  = int(((i+1)/ROW_LENGTH) * CAMERA_WIDTH) - 1.0
            
            square_bgr = mask_bgr[startRow:stopRow, startCol:stopCol]
            square_hsv = mask_hsv[startRow:stopRow, startCol:stopCol]

            # Mean of all the values in rectangular "square" array
            MeanValuesBGR[j, i] = np.mean(square_bgr)
            MeanValuesHSV[j, i] = np.mean(square_hsv)

    # Find index of block with largest number of white pixels N.B. Black = 0, White = 255(?)
    
    # print("The mean values array: ", MeanValues)
    # sizeOfMeanValues = MeanValues.size
    # print("The size of the MeanValues array is: ", str(sizeOfMeanValues))
    # print("The MeanValues array is: ", str(MeanValues))
    
    # Square numbering is top left to top right and then looping down rows
    bigSquareBGR = np.argmax(MeanValuesBGR)
    bigSquareHSV = np.argmax(MeanValuesHSV)

    # Work out off and even for left or right
    # Only works for 2x2 grid currently! Will need to change for other grid sizes
    # N.B. Our left is the robots right
    
##    if (bigSquareBGR % 2) > 0:
##        print("The BGR blue object is to your right")
##    else:
##        print("The BGR blue object is to your left")
##
##    if (bigSquareHSV % 2) > 0:
##        print("The HSV blue object is to your right")
##    else:
##        print("The HSV blue object is to your left")

    # Try using circle detection to filer out noise
    # https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/
    # https://www.pyimagesearch.com/2014/07/21/detecting-circles-images-using-opencv-hough-circles/
    # http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_houghcircles/py_houghcircles.html

    # Or use contours
    # https://www.piborg.org/blog/build/diddyborg-v2-build/diddyborg-v2-examples-ball-following

    # http://picamera.readthedocs.io/en/release-1.10/api_array.html
    # Clear the stream in preperation for the next frame
    rawCapture.truncate(0)

    # Capture a key press. The function waits argument in ms for any keyboard event
    key = cv2.waitKey(1) & 0xFF

    # if the 'q' key was pressed break from the loop
    if key == ord("q"):
        break
    # if the 'c' key was pressed change the colour array counter
    elif key == ord("c"):
        colourArrayCntr = (colourArrayCntr + 1) % 4 # Loop over integers 0 to 3
        print("The colour selector is now ", str(COLOUR_NAME_ARRAY[colourArrayCntr]))

# simply destroys all windows created
# Can use cv2.destroyWindow(frameName) to destroy a specific window
cv2.destroyAllWindows()
