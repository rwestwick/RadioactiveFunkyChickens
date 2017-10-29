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
ROW_LENGTH = 2 # Number of rectangles per row for analysis
COL_LENGTH = 2 # Number of rectangles per column for analysis
MeanValues = np.ones([COL_LENGTH, ROW_LENGTH]) * 255

# Capture frames from the camera
# http://picamera.readthedocs.io/en/release-1.10/api_camera.html
# capture_continuous(output, format=None, use_video_port=False, resize=None, splitter_port=0, burst=False, **options)
# The format, use_video_port, splitter_port, resize, and options parameters are the same as in capture()

# This appears to be importing to BGR not RGB
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array respresenting the image, then intialize the timestap
    # and occupied/unoccupied text
    image = frame.array

    trueColor = image
    
    # Define the colour boundaries in BGR
    lower = [17, 15, 100]
    upper = [100, 100, 255]


    # Create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")

    # find the colours within the specified boundaries and apply the mask
    mask = cv2.inRange(trueColor, lower, upper)
    output = cv2.bitwise_and(trueColor, trueColor, mask = mask)

    # Create a frame for lower middle part of video
    # Top left is [0, 0] in [rows, columns]

    startRow = int(0.90 * CAMERA_HEIGHT)
    stopRow  = int(1.00 * CAMERA_HEIGHT) - 1.0
    startCol = int(0.40 * CAMERA_WIDTH)
    stopCol  = int(0.60 * CAMERA_WIDTH) - 1.0
    
    # show the frame
    cv2.imshow("ColourFrame", trueColor)
    cv2.imshow("Mask", mask)
    cv2.imshow("ColourThreshold", output)

    # Capture a key press. The function waits argument in ms for any keyboard event
    key = cv2.waitKey(1) & 0xFF

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
            
            square = mask[startRow:stopRow, startCol:stopCol]

            # Mean of all the values in rectangular "square" array
            MeanValues[j, i] = np.mean(square)

    # Find index of block with largest number of white pixels N.B. Black = 0, White = 255(?)
    
    # print("The mean values array: ", MeanValues)
    # sizeOfMeanValues = MeanValues.size
    # print("The size of the MeanValues array is: ", str(sizeOfMeanValues))
    # print("The MeanValues array is: ", str(MeanValues))
    
    # Square numbering is top left to top right and then looping down rows
    bigWhiteSquare = np.argmax(MeanValues)
    print(str(bigWhiteSquare))

    # Work out off and even for left or right
    # Only works for 2x2 grid currently! Will need to change for other grid sizes
    # N.B. Our left is the robots right
    
    if (bigWhiteSquare % 2) > 0:
        print("The red object is to your right")
    else:
        print("The red object is to your left")

    # http://picamera.readthedocs.io/en/release-1.10/api_array.html
    # Clear the stream in preperation for the next frame
    rawCapture.truncate(0)

    # if the 'q' key was pressed break from the loop
    if key == ord("q"):
        break

# simply destroys all windows created
# Can use cv2.destroyWindow(frameName) to destroy a specific window
cv2.destroyAllWindows()
