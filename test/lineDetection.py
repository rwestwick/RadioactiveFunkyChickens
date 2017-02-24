# Import libraries
import io
import time
import picamera
import picamera.array
import cv2
import numpy as np

# Create an in-memory stream
my_stream = io.BytesIO()

# Define camera variables
camera_width = 320
camera_height = 240

# How long does the process take
start_time = time.time()

with picamera.PiCamera() as camera:
    camera.resolution = (camera_width, camera_height)
    camera.start_preview(fullscreen=False, window=(100, 20, 320, 240))

    print(time.time() - start_time)

    # Camera warm-up time
    time.sleep(2)

    # Capture into stream
    # camera.capture(my_stream, format='jpeg', use_video_port=True)
    camera.capture(my_stream, format='rgb', use_video_port=True)

# Convert image into numpy array
# data = np.fromstring(my_stream.getvalue(), dtype=np.uint8)

# Convert directly to cv2 image
fwidth = (camera_width + 31) // 32 * 32
fheight = (camera_height + 15) // 16 * 16
if len(my_stream.getvalue()) != (fwidth * fheight * 3):
    raise PiCameraValueError(
        'Incorrect buffer length for resolution %dx%d' (
            camera_width,
            camera_height))
img2 = np.frombuffer(my_stream.getvalue(), dtype=np.uint8).reshape(
    (fheight, fwidth, 3))[:camera_height, :camera_width, :]

# Turn the array into a cv2 image
# img = cv2.imdecode(data, cv2.IMREAD_GRAYSCALE)

# Show number of rows, columns and channels (if image is in colour)
# print img2.shape

# Show image
# cv2.imshow('image', img2)
#
# k = cv2.waitKey(0) &0xFF
# if k == 27: # wait for ESC key to exit
# cv2.destroyAllWindows()

# Convert color to gray
img3 = cv2.cvtColor(img2, cv2.COLOR_RGB2GRAY)

# print img3.shape

# Show image
# cv2.imshow('image', img3)
#
# k = cv2.waitKey(0) &0xFF
# if k == 27: # wait for ESC key to exit
# cv2.destroyAllWindows()

# Adaptive thresholding
threshImgAd = cv2.adaptiveThreshold(
    img3,
     255,
     cv2.ADAPTIVE_THRESH_MEAN_C,
     cv2.THRESH_BINARY,
     11,
     2)
# threshImgAd = cv2.adaptiveThreshold(img3, 255,
# cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

# Break picture into ROI grid and score bottom two lines
num_columns = 5
num_rows = 5
block_width = camera_width / num_columns
block_height = camera_height / num_rows

for row in range(1, 3):
    for column in range(1, 6):
        height_start = camera_height - (row * block_height)
        height_stop = camera_height - ((row - 1) * block_height)
        width_start = (column - 1) * block_width
        width_stop = column * block_width
        block = threshImgAd[height_start:height_stop, width_start:width_stop]
        print 'The mean of all the values: ', np.mean(block)


# Show thresholded image
cv2.imshow('image', threshImgAd)

k = cv2.waitKey(0) & 0xFF
if k == 27:  # wait for ESC key to exit
    cv2.destroyAllWindows()
