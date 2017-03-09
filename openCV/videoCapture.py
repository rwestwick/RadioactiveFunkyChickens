from picamera import PiCamera
from time import sleep

camera = PiCamera()

# Initialize the camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
ROW_LENGTH = 20
camera.resolution = (CAMERA_WIDTH, CAMERA_HEIGHT)
camera.framerate = 10
camera.vflip = True

camera.start_preview()
camera.start_recording('/home/pi/video.h264')
sleep(5)
camera.stop_recording()
camera.stop_preview
