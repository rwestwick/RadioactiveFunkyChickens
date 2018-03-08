#!/usr/bin/python
# SomewhereOverTheRainbow.pw
# http://piwars.org/2018-competition/challenges/somewhere-over-the-rainbow/
"""
This algorithm is designed to solve the Somewhere Over the Rainbow Pi Wars 4.0
challenge.
"""

# https://www.python.org/dev/peps/pep-0238/
# The future division statement, spelled "from __future__ import division",
# will change the / operator to mean true division throughout the module.
# This is needed for the row and column calculations for rectangle arrays to
# prevent rounding down to zero.
from __future__ import division

# Import needed libraries such as picamera OpenCV and NumPy
import logging
import time
import math
import sys
import SetupConsoleLogger
import ServoController
import SpeedSettings
import UltrasonicSensor
import ColourBoundaries
import GPIOLayout
import KeyboardCharacterReader
import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera import PiCamera
from picamera.array import PiRGBArray

# Additional functions defined

def camera_centre_check():
    # Initial pan/tilt servo angles
    pVal = PAN_INTIAL
    tVal = TILT_INTIAL
    SERVO_CONTROLLER.set_pan_servo(pVal)
    SERVO_CONTROLLER.set_tilt_servo(tVal)

    # Manually recentre servos if needed
    LOGGER.info("Press 'w', 'z', 's' and 'a' keys to "
                "recentre pan/tilt servos.")
    LOGGER.info("Press 'n' to start main "
                "'Somewhere Over the Rainbow' algorithm.")

    for frameServo in camera.capture_continuous(
            rawCaptureServo, format="bgr", use_video_port=True):
        # Grab the raw NumPy array representing the image,
        # then initialize the timestamps and occupied/unoccupied text
        imageServo = frameServo.array

        # Show Pan and Tilt values on screen
        imageTextString1 = 'Pan = ' + str(pVal) + ' Tilt = ' + str(tVal)
        cv2.putText(imageServo, imageTextString1, (50, 20), FONT, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)

        # Show the frame(s)
        cv2.imshow("Camera View", imageServo)

        # Capture a key press.
        # The function waits in ms for any keyboard event
        key = cv2.waitKey(100) & 0xFF

        if key == ord("w"):
            pVal = min(90, pVal + 10)
            SERVO_CONTROLLER.set_pan_servo(pVal)
            LOGGER.info("Pan/Tilt servos up.")

        elif key == ord("z"):
            pVal = max(-90, pVal - 10)
            SERVO_CONTROLLER.set_pan_servo(pVal)
            LOGGER.info("Pan/Tilt servos down.")

        elif key == ord("s"):
            tVal = min(90, tVal + 10)
            SERVO_CONTROLLER.set_tilt_servo(tVal)
            LOGGER.info("Pan/Tilt servos right.")

        elif key == ord("a"):
            tVal = max(-90, tVal - 10)
            SERVO_CONTROLLER.set_tilt_servo(tVal)
            LOGGER.info("Pan/Tilt servos left.")

        elif key == ord("n"):
            LOGGER.info("Stopped centring servos.")
            break

        # Clear the stream in preparation for the next frame
        rawCaptureServo.truncate(0)

    # Close Servo Setting window
    cv2.destroyAllWindows()


def contour_circularity(cnts):
    # Compute the circularity of the contours in the array
    # The lower the value the less circular
    # Perfect circle has value of one

    # Initialize the circularity array
    circularityArray = []

    # Calculate circularity for each contour in input array
    for c in cnts:
        AreaContour = cv2.contourArea(c)
        Perimeter = cv2.arcLength(c, True)
        if Perimeter != 0.0:
            circularity = (4 * math.pi * AreaContour) / \
                (math.pow(Perimeter, 2))
        else:
            circularity = 0

        circularityArray.append(circularity)

    # return an array of circularity values
    return circularityArray


def contour_centre(cntr):
    # compute the centre of the contour area
    M = cv2.moments(cntr)

    # Prevent division by 0
    if M["m00"] == 0:
        x, y, w, h = cv2.boundingRect(cntr)
        cX = x + (w / 2)
        cY = y + (h / 2)
    else:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

    # return the x and y co-ordinates of the centre of contours
    return cX, cY


def find_HSV_colour(colourArrayCntr, bgrImage):
    # Find chosen colours in video image using HSV
    # Inputs:
    # colourArrayCntr - Integer for selection of upper and lower colour bands
    # bgrImage - BGR image from camera
    # Outputs:
    # Masked BGR image and its mask from colour detection
    # Masked blurred BGR image and its mask from colour detection

    # Select HSV colour range boundaries to detect marker
    lower_hsv = ColourBoundaries.LOWER_HSV_ARRAY[colourArrayCntr]
    upper_hsv = ColourBoundaries.UPPER_HSV_ARRAY[colourArrayCntr]
    lower_red_lft_hsv = ColourBoundaries.LOWER_RED_LFT_HSV
    upper_red_lft_hsv = ColourBoundaries.UPPER_RED_LFT_HSV

    # Create HSV NumPy arrays from the boundaries
    lower_hsv = np.array(lower_hsv, dtype="uint8")
    upper_hsv = np.array(upper_hsv, dtype="uint8")
    lower_red_lft_hsv = np.array(lower_red_lft_hsv, dtype="uint8")
    upper_red_lft_hsv = np.array(upper_red_lft_hsv, dtype="uint8")

    # Convert BGR to HSV
    hsvImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2HSV)

    # Find the colours within the specified boundaries and apply the mask
    mask_hsv = cv2.inRange(hsvImage, lower_hsv, upper_hsv)
    if colourArrayCntr == 0:
        mask_hsv = mask_hsv + cv2.inRange(hsvImage, lower_red_lft_hsv,
                                          upper_red_lft_hsv)

    # Applying mask to BGR image gives true colours on display
    output_hsv = cv2.bitwise_and(bgrImage, bgrImage, mask=mask_hsv)

    # https://www.piborg.org/blog/build/diddyborg-v2-build/diddyborg-v2-examples-ball-following
    # https://docs.opencv.org/3.0-beta/modules/imgproc/doc/filtering.html
    # cv2.medianBlur(src, ksize[, dst]) -> dst
    # Smooths an image using the median filter with
    # the ksize * ksize aperture
    # Computes the median of all the pixels
    # Should blurring happen in BGR or HSV?
    bgrImageBlur = cv2.medianBlur(bgrImage, MED_FILTER_APRTRE_SIZE)
    hsvImageBlur = cv2.cvtColor(bgrImageBlur, cv2.COLOR_BGR2HSV)

    # Find the colour in the blurred image
    mask_hsvImageBlur = cv2.inRange(hsvImageBlur, lower_hsv, upper_hsv)
    if colourArrayCntr == 0:
        mask_hsvImageBlur = mask_hsvImageBlur + cv2.inRange(
            hsvImage, lower_red_lft_hsv, upper_red_lft_hsv)

    output_hsvImageBlur = cv2.bitwise_and(
        bgrImageBlur, bgrImageBlur, mask=mask_hsvImageBlur)

    imageTextString2 = 'Colour = ' + \
                       ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr]

    cv2.putText(output_hsv, imageTextString2, (50, 40), FONT, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(output_hsvImageBlur, imageTextString2, (50, 40), FONT, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)

    return output_hsv, mask_hsv, output_hsvImageBlur, mask_hsvImageBlur


def find_YUV_colour(colourArrayCntr, bgrImage):
    # Find chosen colours in video image using YUV
    # Inputs:
    # colourArrayCntr - Integer for selection of upper and lower colour bands
    # bgrImage - BGR image from camera
    # Outputs:
    # Masked BGR image and its mask from colour detection
    # Masked blurred BGR image and its mask from colour detection

    # Select YUV colour range boundaries to detect marker
    lower_yuv = ColourBoundaries.LOWER_YUV_ARRAY[colourArrayCntr]
    upper_yuv = ColourBoundaries.UPPER_YUV_ARRAY[colourArrayCntr]

    # Create YUV NumPy arrays from the boundaries
    lower_yuv = np.array(lower_yuv, dtype="uint8")
    upper_yuv = np.array(upper_yuv, dtype="uint8")

    # Convert BGR to YUV
    yuvImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2YUV)

    # Find the colours within the specified boundaries and apply the mask
    mask_yuv = cv2.inRange(yuvImage, lower_yuv, upper_yuv)

    # Applying mask to BGR image gives true colours on display
    output_yuv = cv2.bitwise_and(bgrImage, bgrImage, mask=mask_yuv)

    # https://www.piborg.org/blog/build/diddyborg-v2-build/diddyborg-v2-examples-ball-following
    # https://docs.opencv.org/3.0-beta/modules/imgproc/doc/filtering.html
    # cv2.medianBlur(src, ksize[, dst]) -> dst
    # Smooths an image using the median filter with
    # the ksize * ksize aperture
    # Computes the median of all the pixels
    # Should blurring happen in BGR or YUV?
    bgrImageBlur = cv2.medianBlur(bgrImage, MED_FILTER_APRTRE_SIZE)
    yuvImageBlur = cv2.cvtColor(bgrImageBlur, cv2.COLOR_BGR2YUV)

    # Find the colour in the blurred image
    mask_yuvImageBlur = cv2.inRange(yuvImageBlur, lower_yuv, upper_yuv)

    output_yuvImageBlur = cv2.bitwise_and(
        bgrImageBlur, bgrImageBlur, mask=mask_yuvImageBlur)

    imageTextString2 = 'Colour = ' + \
                       ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr]

    cv2.putText(output_yuv, imageTextString2, (50, 40), FONT, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(output_yuvImageBlur, imageTextString2, (50, 40), FONT, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)

    return output_yuv, mask_yuv, output_yuvImageBlur, mask_yuvImageBlur


def find_marker_contour(mask, output_hsv):
    # Find location of marker centre
    # Inputs:
    # mask - Mask from colour detection
    # output_hsv - BGR image for modification
    # Outputs:
    # output_hsv - BGR image with added text from analysis results
    # contourDetection - True if contours detected otherwise False
    # foundX - Location of centre of contour in pixels from left of image
    # foundY - Location of centre of contour in pixels from top of image

    # Find the contours
    # RETR_TREE works, but is not in piborg example which uses RETR_LIST
    # RETR_EXTERNAL does not look for contours within contours
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(output_hsv, contours, -1, (0, 255, 0), 3)

    # Calculate final number of largest area contours
    if len(contours) == 0:
        LOGGER.info("No contours found.")
    elif len(contours) < NUM_OF_LARGEST_AREA_CONTOURS:
        finalNumLargestAreaContours = len(contours)
    else:
        finalNumLargestAreaContours = NUM_OF_LARGEST_AREA_CONTOURS

    # Sort for three largest contours by area
    cntSortedByArea = sorted(
        contours, key=cv2.contourArea,
        reverse=True)[:finalNumLargestAreaContours]

    # Highlight largest contours by area in Yellow even
    # if smaller than Min area
    cv2.drawContours(output_hsv, cntSortedByArea, -1, (0, 255, 255), 3)

    # Calculate position of contour that still is greater than Min area
    cntWithMinArea = []
    for cntCounterArea in range(len(cntSortedByArea)):
        if cv2.contourArea(cntSortedByArea[cntCounterArea]) \
           >= MIN_MARKER_AREA:
            cntWithMinArea.append(cntSortedByArea[cntCounterArea])
        else:
            break

    # Check to see if any contours are found as circularity and
    # zip does not work without array
    if len(cntWithMinArea) == 0:
        contourDetection = False
        cv2.putText(output_hsv, 'No contours found!', (50, 140), FONT, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
    else:
        contourDetection = True

        # Calculate the largest contours' by area circularity value
        cntCircularity = contour_circularity(cntWithMinArea)

        # Sort contours in order of circularity
        (cntSortedByCirc, cntCircularity) = zip( * sorted(
            zip(cntWithMinArea, cntCircularity),
            key=lambda x: x[1],
            reverse=True))

        # Highlight the most circular contour in white
        cv2.drawContours(output_hsv, cntSortedByCirc[0], -1, (255, 255, 255),
                         3)

        # Calculate centre of most circular contour
        foundX, foundY = contour_centre(cntSortedByCirc[0])

        # Show location of centre of largest contour as white dot
        cv2.circle(output_hsv, (int(foundX), int(foundY)), 10, (255, 255, 255),
                   -1)

        # Show area and circularity of chosen contour
        imageTextString5 = "Area = " + \
            str(round(cv2.contourArea(cntSortedByCirc[0]), 2))

        cv2.putText(output_hsv, imageTextString5, (50, 140), FONT, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)

        imageTextString6 = "Circularity = " + \
                           str(round(cntCircularity[0], 2))

        cv2.putText(output_hsv, imageTextString6, (50, 160), FONT, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)

    # Check to see if top three contours are greater than
    # minimum circularity to prevent false hits
    imageTextString4 = 'Contour(s) '
    contoursWithinSizeAndCircularity = True

    for cntCounter in range(len(cntSortedByCirc)):
        currentCntrCircularity = cntCircularity[cntCounter]
        if (currentCntrCircularity < MIN_MARKER_CIRCULARITY):
            # Contours numbered 1 to finalNumLargestAreaContours
            imageTextString4 = imageTextString4 + str(cntCounter + 1) + ' '
            contoursWithinSizeAndCircularity = False

    if contoursWithinSizeAndCircularity:
        imageTextString7 = "Top " + str(len(cntSortedByCirc)) + \
            " contours within size and circularity boundary."
        cv2.putText(output_hsv, imageTextString7, (50, 120), FONT, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
    else:
        imageTextString4 = imageTextString4 + 'of ' + \
            str(len(cntSortedByCirc)) + \
            ' are not circular enough'
        cv2.putText(output_hsv, imageTextString4, (50, 120), FONT, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)

    return output_hsv, contourDetection, foundX, foundY

# Initialise objects and constants

# Create a logger to both file and stdout
LOGGER = logging.getLogger("__name__")
SetupConsoleLogger.setup_console_logger(LOGGER)

# Set movement constant values
FRONT_BUFFER_WARN = 35  # Shortest distance to front (cm)
FRONT_BUFFER_STOP = 20  # Shortest distance to front (cm)
SIDE_BUFFER = 10  # Shortest distance to side (cm)
CORRECTION_TIME = 0.15  # Angle correction delay time in seconds
FORWARD_TIME = 0.05  # Forward time iteration delay time in seconds
TURN_DELAY = 0.65  # Delay when turning in seconds
PAN_INTIAL = 20  # Initial pan angle in degrees
TILT_INTIAL = 20  # Initial tilt angle in degrees

# Initialise motors
ROBOTMOVE = DualMotorController.DualMotorController(
    GPIOLayout.MOTOR_LEFT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_FRONT_BACKWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_LEFT_REAR_BACKWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_FORWARD_GPIO,
    GPIOLayout.MOTOR_RIGHT_REAR_BACKWARD_GPIO)

# Initialise servos
SERVO_CONTROLLER = ServoController.ServoController()

# Initialise camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
camera = PiCamera()  # Initialize camera
# Camera resolution defaults to the monitors resolution,
# but needs to be lower for speed of processing
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
camera.framerate = 10  # If not set then defaults to 30fps
camera.vflip = True  # Needed for mounting of camera on pan/tilt
camera.hflip = True  # Needed for mounting of camera on pan/tilt

# Set FONT for text on image/video
FONT = cv2.FONT_HERSHEY_COMPLEX_SMALL

# Image filtering constants
MED_FILTER_APRTRE_SIZE = 5  # Aperture size for median filter
# Initial number for down selecting large contours
# If number too small then will loose circular marker
NUM_OF_LARGEST_AREA_CONTOURS = 3
MIN_MARKER_AREA = 100  # Pixels - the final value to be decided from testing
MIN_MARKER_CIRCULARITY = 0.5  # Correct value to be decided

# Initialize tracking columns
NUM_COLS = 5  # Number linked to column constants so cannot be changed!
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
    sys.exit()

# http://picamera.readthedocs.io/en/release-1.10/api_array.html
# class picamera.array.PiRGBArray(camera, size=None)[source]
# Produces a 3-dimensional RGB array from an RGB capture with the dimensions
# (rows, columns, plane), for example of size (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
# Video capture for servo pan/tilt setting
rawCaptureServo = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Video capture for main algorithm
rawCapture = PiRGBArray(camera, size=(CAMERA_WIDTH, CAMERA_HEIGHT))

# Allow the camera time to warm-up
time.sleep(1)


def main():
    """
    Performs the "Somewhere Over the Rainbow" algorithm
    Method 1 - First choice
    Method 2 - Emergency backup
    """

    LOGGER.info("Somewhere Over The Rainbow")

    # Start servos
    LOGGER.info("Start Pan/Tilt servos")
    SERVO_CONTROLLER.start_servos()
    time.sleep(1)

    # Set initial pan and tilt values then manual recentre if needs be
    camera_centre_check()

    # Set initial colour from COLOUR_NAME_ARRAY array position -
    # 'Red', 'Blue', 'Green', 'Yellow'
    colourArrayCntr = 0

    # Initialize photo capture
    imageNum = 1

    # Show commands and status
    LOGGER.info("Press 'q' to quit.")
    LOGGER.info("Press 'c' to change colour selector.")
    LOGGER.info("Press 'p' to take picture of current frame.")
    LOGGER.info("All key presses must be in a video frame window.")
    LOGGER.info("The colour selector is now " +
                ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr])

    # Waiting for start of challenge
    LOGGER.info("Press 'Space' in console to start.")

    # Create necessary sensor objects
    view_left = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_LEFT_RX_GPIO, GPIOLayout.SONAR_LEFT_TX_GPIO)
    view_right = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_RIGHT_RX_GPIO, GPIOLayout.SONAR_RIGHT_TX_GPIO)
    view_front = UltrasonicSensor.UltrasonicSensor(
        GPIOLayout.SONAR_FRONT_TX_GPIO)

    LOGGER.info("Distance view_left at start " +
                format(view_left.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_right at start " +
                format(view_right.measurement(), '.2f') + " cm")
    LOGGER.info("Distance view_front at start " +
                format(view_front.measurement(), '.2f') + " cm")

    # Start the challenge when space key presses
    while True:
        keyp = KeyboardCharacterReader.readkey()
        if keyp == ' ':
            LOGGER.info("Go")
            break

    # Capture frames from the camera
    # http://picamera.readthedocs.io/en/release-1.10/api_camera.html
    # capture_continuous(output, format=None, use_video_port=False,
    # resize=None, splitter_port=0, burst=False, **options)
    # The format, use_video_port, splitter_port, resize, and
    # options parameters are the same as in capture()

    for frame in camera.capture_continuous(
            rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image,
        # then initialize the time stamp and occupied/unoccupied text
        bgrImage = frame.array

        # Find chosen colour in image
        output_hsv, mask_hsv, output_hsvImageBlur, mask_hsvImageBlur = find_HSV_colour(
            colourArrayCntr, bgrImage)
        # output_hsv, mask_hsv, output_hsvImageBlur, mask_hsvImageBlur = find_YUV_colour(
        #    colourArrayCntr, bgrImage)

        # Find location of contour
        output_hsv, contourDetection, foundX, foundY = find_marker_contour(
            mask_hsvImageBlur, output_hsvImageBlur)

        # Change speed depending on distance to front wall
        distanceToFrontWall = view_front.measurement()
        distanceToLeftWall = view_left.measurement()
        distanceToRightWall = view_right.measurement()
        imageTextString3 = 'Distance to front, left and right walls = ' + \
                           str(round(distanceToFrontWall, 2)) + ', ' + \
                           str(round(distanceToLeftWall, 2)) + ', ' + \
                           str(round(distanceToRightWall, 2)) + 'cm'
        cv2.putText(output_hsv, imageTextString3, (50, 80), FONT, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)

        if distanceToFrontWall > FRONT_BUFFER_WARN:
            cv2.putText(output_hsv, 'Full speed.', (50, 100), FONT, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)
            speedForward = SpeedSettings.SPEED_FAST
        elif FRONT_BUFFER_WARN >= distanceToFrontWall >= FRONT_BUFFER_STOP:
            cv2.putText(output_hsv, 'Slowly now.', (50, 100), FONT, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)
            speedForward = SpeedSettings.SPEED_SLOW
        elif distanceToFrontWall < FRONT_BUFFER_STOP:
            cv2.putText(output_hsv, 'Breaks on.', (50, 100), FONT, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)
            speedForward = 0
            ROBOTMOVE.stop()
        else:
            imageTextString8 = 'Something funny with measurements. ' \
                               'Boundaries are ' + str(FRONT_BUFFER_WARN) + \
                               ' and ' + str(FRONT_BUFFER_STOP) + ' cm'
            cv2.putText(output_hsv, imageTextString8, (50, 100), FONT, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)
            speedForward = 0
            ROBOTMOVE.stop()

        if contourDetection and speedForward != 0:
            # Work out whether to turn left or right from contour position
            if FAR_LEFT_COL_XLINE1 <= foundX < FAR_LEFT_COL_XLINE2:
                cv2.putText(output_hsv, 'Turn fast left.', (50, 60), FONT, 0.6,
                            (255, 255, 255), 1, cv2.LINE_AA)
                ROBOTMOVE.turn_forward(0, speedForward)
                time.sleep(FORWARD_TIME)
            elif LEFT_COL_XLINE1 <= foundX < LEFT_COL_XLINE2:
                cv2.putText(output_hsv, 'Turn left.', (50, 60), FONT, 0.6,
                            (255, 255, 255), 1, cv2.LINE_AA)
                ROBOTMOVE.turn_forward((speedForward / 2), speedForward)
                time.sleep(FORWARD_TIME)
            elif CNTR_COL_XLINE1 >= foundX < CNTR_COL_XLINE2:
                cv2.putText(output_hsv, 'Straight on.', (50, 60), FONT, 0.6,
                            (255, 255, 255), 1, cv2.LINE_AA)
                ROBOTMOVE.forward(speedForward)
                time.sleep(FORWARD_TIME)
            elif RIGHT_COL_XLINE1 <= foundX < RIGHT_COL_XLINE2:
                cv2.putText(output_hsv, 'Turn right.', (50, 60), FONT, 0.6,
                            (255, 255, 255), 1, cv2.LINE_AA)
                ROBOTMOVE.turn_forward(speedForward, (speedForward / 2))
                time.sleep(FORWARD_TIME)
            elif FAR_RIGHT_COL_XLINE1 <= foundX < FAR_RIGHT_COL_XLINE2:
                cv2.putText(output_hsv, 'Turn fast right.', (50, 60), FONT,
                            0.6, (255, 255, 255), 1, cv2.LINE_AA)
                ROBOTMOVE.turn_forward(speedForward, 0)
                time.sleep(FORWARD_TIME)
        else:
            cv2.putText(output_hsv, 'Too close to turn.', (50, 60), FONT, 0.6,
                        (255, 255, 255), 1, cv2.LINE_AA)

        # Spin robot to work out position of each coloured marker

        # Point towards Red marker and go into quarter circle zone

        # Point towards Blue marker and go into quarter circle zone

        # Point towards Yellow marker and go into quarter circle zone

        # Point towards Green marker and go into quarter circle zone

        # Show the frame(s)
        # cv2.imshow("BGR ColourFrame", bgrImage)
        # cv2.imshow("HSV Mask", mask_hsv)
        # cv2.imshow("HSV Mask Image Blur", mask_hsvImageBlur)
        cv2.imshow("HSV ColourThreshold", output_hsv)
        # cv2.imshow("Blurred HSV ColourThreshold", output_hsvImageBlur)

        # http://picamera.readthedocs.io/en/release-1.10/api_array.html
        # Clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # Capture a key press. The function waits argument in ms
        # for any keyboard event
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key was pressed break from the loop
        if key == ord("q"):
            break
        # if the 'c' key was pressed change the colour array counter
        elif key == ord("c"):
            # Loop over integers 0 to 3
            colourArrayCntr = (colourArrayCntr + 1) % 4
            LOGGER.info("The colour selector is now " +
                        ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr])
        # if the 'p' key was pressed capture the images
        elif key == ord("p"):
            fileNameBGR = 'bgrImage' + str(imageNum) + '.png'
            fileNameMaskHSV = 'hsvImageMask' + \
                              str(ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr]) + \
                              str(imageNum) + '.png'
            fileNameOutputHSV = 'hsvImageOutput' + \
                                str(ColourBoundaries.COLOUR_NAME_ARRAY[colourArrayCntr]) + \
                                str(imageNum) + '.png'
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
