# Import OpenCV and NumPy
import cv2
import numpy as np

# Read image file as grayscale
img = cv2.imread('image.jpg', cv2.IMREAD_GRAYSCALE)

# Show image
cv2.imshow('image', img)

k = cv2.waitKey(0) &0xFF
if k == 27: # wait for ESC key to exit
    cv2.destroyAllWindows()
    
# Access individual pixel
px = img[100, 100]

# px is an array of Blue, Green, Red values
print px

# Show number of rows, columns and channels (if image is in colour)
print img.shape

# Image region of interest (ROI)
square = img[200:300, 300:400]

# Show square ROI
cv2.imshow('image', square)

k = cv2.waitKey(0) &0xFF
if k == 27: # wait for ESC key to exit
    cv2.destroyAllWindows()

# Small ROI
tidbit = img[200:205, 200:205]
print tidbit

# Sum and mean of all the values in tidbit array
print 'The sum of all the values: ', np.sum(tidbit)
print 'The mean of all the values: ', np.mean(tidbit)

# Simple thresholding
ret,threshImg = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

# Show thresholded image
cv2.imshow('image', threshImg)

k = cv2.waitKey(0) &0xFF
if k == 27: # wait for ESC key to exit
    cv2.destroyAllWindows()

# Adaptive thresholding
threshImgAd = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
# threshImgAd = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

# Show thresholded image
cv2.imshow('image', threshImgAd)

k = cv2.waitKey(0) &0xFF
if k == 27: # wait for ESC key to exit
    cv2.destroyAllWindows()
