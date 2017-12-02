#!/usr/bin/python
# SomewhereOverTheRainbow.pw
# http://piwars.org/2018-competition/challenges/somewhere-over-the-rainbow/

"""
This algorithm is designed to solve the Somewhere Over the Rainbow Pi Wars 4.0
challenge.
"""

# https://www.python.org/dev/peps/pep-0238/
# The future division statement, spelled "from __future__ import division", will change the / operator to mean true division
# throughout the module.
# This is needed for the row and column calculations for rectangle arrays to prevent rounding down to zero.
from __future__ import division

# Import needed libraries such as picamera OpenCV and NumPy
import logging
import time
import SetupConsoleLogger
import ServoController
import MotorController
import UltrasonicSensor
import GPIOLayout
import KeyboardCharacterReader
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

# Create a logger to both file and stdout
LOGGER = logging.getLogger("__name__")
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set initial constant values
FRONT_BUFFER_WARN = 35 # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 20 # Shortest distance to front (cm)
SIDE_BUFFER = 10 # Shortest distance to side (cm)
CORRECTION_TIME = 0.15 # Angle correction delay time in seconds
FORWARD_TIME = 0.1 # Angle correction delay time in seconds
TURN_DELAY = 0.65
PAN_INTIAL = -30
TILT_INTIAL = 0

# Define the colour boundaries in HSV
LOWER_RED_LFT_HSV = [165, 100, 100] # Left of 0deg Red = ~330deg to 359deg
UPPER_RED_LFT_HSV = [179, 255, 255] # Red
LOWER_RED_HSV = [0, 100, 100] # Red = 0deg to ~30deg
UPPER_RED_HSV = [15, 255, 255] # Red
LOWER_BLUE_HSV = [80, 50, 50] # Blue = ~180deg to ~260deg
UPPER_BLUE_HSV = [140, 255, 255] # Blue
LOWER_GREEN_HSV = [45, 50, 50] # Green = ~90deg to ~150deg
UPPER_GREEN_HSV = [75, 255, 255] # Green
LOWER_YELLOW_HSV = [20, 100, 100] # Yellow = ~40deg to ~90deg
UPPER_YELLOW_HSV = [45, 255, 255] # Yellow
LOWER_HSV_ARRAY = [LOWER_RED_HSV, LOWER_BLUE_HSV, LOWER_GREEN_HSV, LOWER_YELLOW_HSV]
UPPER_HSV_ARRAY = [UPPER_RED_HSV, UPPER_BLUE_HSV, UPPER_GREEN_HSV, UPPER_YELLOW_HSV]

# Initialize colour array counter
COLOUR_NAME_ARRAY = ['Red', 'Blue', 'Green', 'Yellow']

# Initialise motors
ROBOTMOVE = MotorController.MotorController(
    GPIOLayout.MOTOR_LEFT_FORWARD_PIN,
    GPIOLayout.MOTOR_LEFT_BACKWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_FORWARD_PIN,
    GPIOLayout.MOTOR_RIGHT_BACKWARD_PIN)

# Initialise servos
SERVO_CONTROLLER = ServoController.ServoController()

# Initialize camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
camera = PiCamera() # Initialize camera
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT) # resolution defaults to dosplays resolution
# Can get framerates up to 60fps 640x480
camera.framerate = 10 # If not set then defaults to 30fps
camera.vflip = True
camera.hflip = True

# Initialize tracking columns
NUM_COLS = 5 # This number is linked to the column constants so cannot be changed
COL_WIDTH = int(CAMERA_WIDTH / NUM_COLS)
FAR_LEFT_COL_XLINE1 = 0
FAR_LEFT_COL_XLINE2 = FAR_LEFT_COL_XLINE1 + COL_WIDTH
LEFT_COL_XLINE1 = FAR_LEFT_COL_XLINE2
LEFT_COL_XLINE2 = LEFT_COL_XLINE1 + COL_WIDTH
CNTR_COL_XLINE1 = LEFT_COL_XLINE2
CNTR_COL_XLINE2 = CNTR_COL_XLINE1 + COL_WIDTH
RIGHT_COL_XLINE1 = CNTR_COL_XLINE2
RIGHT_COL_XLINE2 = RIGHT_COL_XLINE1 + COL_WIDTH
FAR_RIGHT_COL_XLINE1 = RIGHT_COL_XLINE2
FAR_RIGHT_COL_XLINE2 = FAR_RIGHT_COL_XLINE1 + COL_WIDTH

if FAR_RIGHT_COL_XLINE2 > CAMERA_WIDTH:
    LOGGER.info("Issue with column width calculations!")

# http://picamera.readthedocs.io/en/release-1.10/api_array.html
# class picamera.array.PiRGBArray(camera, size=None)[source]
# Produces a 3-dimensional RGB array from an RGB capture with the dimensions (rows, columns, plane)
# for example of size (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Allow the camera time to warmup
time.sleep(1)

def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    Method 1 - First choice
    Method 2 - Emergency backup
    """

    LOGGER.info("Minimal Maze")

    # Set initial colour from COLOUR_NAME_ARRAY array position - 'Red', 'Blue', 'Green', 'Yellow'
    colourArrayCntr = 0

    # Initialize photo capture
    imageNum = 1

    # Show commands and status
    LOGGER.info("Press 'q' to quit.")
    LOGGER.info("Press 'c' to change colour selector.")
    LOGGER.info("Press 'p' to take picture of current frame.")
    LOGGER.info("All key presses must be in a video frame window.")
    LOGGER.info("The colour selector is now " + COLOUR_NAME_ARRAY[colourArrayCntr])

    # Waiting for start of challenge
    LOGGER.info("To start 'Somewhere Over the Rainbow' press 'Space' key.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_PIN,
        GPIOLayout.SONAR_LEFT_TX_PIN)
    view_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_PIN,
        GPIOLayout.SONAR_RIGHT_TX_PIN)
    view_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_PIN)

    LOGGER.info("Distance view_left at start " + format(view_left.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_right at start " + format(view_right.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_front at start " + format(view_front.measurement(), '.2f') + " cm")

    # Setting the camera to look ahead
    SERVO_CONTROLLER.start_servos()
    time.sleep(1) # Need to add a delay or echo does not work!
    SERVO_CONTROLLER.set_pan_servo(PAN_INTIAL)
    SERVO_CONTROLLER.set_tilt_servo(TILT_INTIAL) 

    # Start the challenge
    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

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
        lower_hsv = LOWER_HSV_ARRAY[colourArrayCntr]
        upper_hsv = UPPER_HSV_ARRAY[colourArrayCntr]

        # Create HSV NumPy arrays from the boundaries
        lower_hsv = np.array(lower_hsv, dtype = "uint8")
        upper_hsv = np.array(upper_hsv, dtype = "uint8")

        # Find the colours within the specified boundaries and apply the mask - HSV
        mask_hsv = cv2.inRange(hsvImage, lower_hsv, upper_hsv)

        # Applying mask to BGR image gives true colours on display
        output_hsv = cv2.bitwise_and(bgrImage, bgrImage, mask = mask_hsv)

        # Or use contours
        # https://www.piborg.org/blog/build/diddyborg-v2-build/diddyborg-v2-examples-ball-following
        # https://docs.opencv.org/3.0-beta/modules/imgproc/doc/filtering.html
        # cv2.medianBlur(src, ksize[, dst]) -> dst
        # smoothes an image using the median filter with the ksize * ksize aperture
        bgrImageBlur = cv2.medianBlur(bgrImage, 5) # Computes the median of all the pixels
        hsvImageBlur = cv2.cvtColor(bgrImageBlur, cv2.COLOR_BGR2HSV)

        # Find the colour in blurried image
        mask_hsvImageBlur = cv2.inRange(hsvImageBlur, lower_hsv, upper_hsv)
        output_hsvImageBlur = cv2.bitwise_and(bgrImageBlur, bgrImageBlur, mask = mask_hsvImageBlur)

        # Find the contours
        # imgray = cv2.cvtColor(output_hsvImageBlur, cv2.COLOR_BGR2GRAY)
        # ret,thresh = cv2.threshold(imgray, 50, 255, cv2.THRESH_BINARY)
        # RETR_TREE works, but is not in piborg example which uses RETR_LIST
        # RETR_EXTERNAL does not look for contours within contours
        im2,contours,hierarchy = cv2.findContours(mask_hsvImageBlur, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
        cv2.drawContours(output_hsv, contours, -1, (0,255,0), 3)

        # Go through each contour
        foundArea = -1
        foundX = -1
        foundY = -1
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + (w/2)
            cy = y + (h/2)
            area = w * h
            if foundArea < area:
                foundArea = area
                foundX = cx
                foundY = cy
        if foundArea > 0:
            ball = [foundX, foundY, foundArea]
        else:
            ball = None

        if ball == None:
            LOGGER.info("No contours.")
        else: # If ball is found show details
            # Show location of centre of largest contour as white dot
            cv2.circle(output_hsv, (int(foundX), int(foundY)), 10, (255, 255, 255), -1)

            # Show largest contour values
            imageTextString1 = 'X = ' + str(foundX) + ' Y = ' + str(foundY)
            imageTextString2 = 'Area = ' + str(foundArea)
            font = cv2.FONT_HERSHEY_COMPLEX_SMALL
            cv2.putText(output_hsv, imageTextString1, (50, 20), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(output_hsv, imageTextString2, (50, 40), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

            # Work out whether to turn left or right from contour position
            if ((foundX >= FAR_LEFT_COL_XLINE1) and (foundX < FAR_LEFT_COL_XLINE2)):
                cv2.putText(output_hsv, 'Turn fast right.', (50, 60), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            elif ((foundX >= LEFT_COL_XLINE1) and (foundX < LEFT_COL_XLINE2)):
                cv2.putText(output_hsv, 'Turn right.', (50, 60), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            elif ((foundX >= CNTR_COL_XLINE1) and (foundX < CNTR_COL_XLINE2)):
                cv2.putText(output_hsv, 'Straight on.', (50, 60), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            elif ((foundX >= RIGHT_COL_XLINE1) and (foundX < RIGHT_COL_XLINE2)):
                cv2.putText(output_hsv, 'Turn left.', (50, 60), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
            elif ((foundX >= FAR_RIGHT_COL_XLINE1) and (foundX < FAR_RIGHT_COL_XLINE2)):
                cv2.putText(output_hsv, 'Turn fast left.', (50, 60), font, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        # Change speed depending on distance to front wall
        distanceToFrontWall = view_front.measurement()
        imageTextString3 = 'Distance = ' + str(distanceToFrontWall)
        fontDistance = cv2.FONT_HERSHEY_COMPLEX_SMALL
        cv2.putText(output_hsv, imageTextString3, (50, 80), fontDistance, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        
        if distanceToFrontWall > FRONT_BUFFER_WARN:
            cv2.putText(output_hsv, 'Full speed.', (50, 100), fontDistance, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        elif ((distanceToFrontWall < FRONT_BUFFER_WARN) and (distanceToFrontWall > FRONT_BUFFER_STOP)):
            cv2.putText(output_hsv, 'Slowly now.', (50, 100), fontDistance, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        elif ((distanceToFrontWall < FRONT_BUFFER_STOP)):
            cv2.putText(output_hsv, 'Breaks on.', (50, 100), fontDistance, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        # Spin robot to work out position of each coloured marker

        # Point towards Red marker and go into quarter cirle zone

        # Point towards Blue marker and go into quarter cirle zone

        # Point towards Yellow marker and go into quarter cirle zone

        # Point towards Green marker and go into quarter cirle zone

        # Show the frame(s)
        cv2.imshow("BGR ColourFrame", bgrImage)
        cv2.imshow("HSV Mask", mask_hsv)
        cv2.imshow("HSV ColourThreshold", output_hsv)

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
            LOGGER.info("The colour selector is now " + COLOUR_NAME_ARRAY[colourArrayCntr])
        # if the 'p' key was pressed capture the images
        elif key == ord("p"):
            fileNameBGR = 'bgrImage' + str(imageNum) + '.png'
            fileNameMaskHSV = 'hsvImageMask' + str(COLOUR_NAME_ARRAY[colourArrayCntr]) + str(imageNum) + '.png'
            fileNameOutputHSV = 'hsvImageOutput' + str(COLOUR_NAME_ARRAY[colourArrayCntr]) + str(imageNum) + '.png'
            imageNum += 1
            cv2.imwrite(fileNameBGR, bgrImage)
            cv2.imwrite(fileNameMaskHSV, mask_hsv)
            cv2.imwrite(fileNameOutputHSV, output_hsv)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        LOGGER.info("Stopping 'Somewhere Over the Rainbow'.")
    finally:
        LOGGER.info("'Somewhere Over the Rainbow' Finished.")
        SERVO_CONTROLLER.stop_servos()
        ROBOTMOVE.cleanup()
        cv2.destroyAllWindows()
        GPIO.cleanup()
        
        
