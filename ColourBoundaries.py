"""
Colour definitions used by the somewhere over the rainbow activity.
"""
# Define the colour boundaries in HSV
LOWER_RED_LFT_HSV = [165, 50, 50]  # Left of 0deg Red = ~330deg to 359deg
UPPER_RED_LFT_HSV = [179, 255, 255]  # Red
LOWER_RED_HSV = [0, 50, 50]  # Red = 0deg to ~30deg
UPPER_RED_HSV = [15, 255, 255]  # Red
LOWER_BLUE_HSV = [80, 50, 50]  # Blue = ~180deg to ~260deg
UPPER_BLUE_HSV = [140, 255, 255]  # Blue
LOWER_GREEN_HSV = [45, 50, 50]  # Green = ~90deg to ~150deg
UPPER_GREEN_HSV = [75, 255, 255]  # Green
LOWER_YELLOW_HSV = [15, 50, 50]  # Yellow = ~30deg to ~90deg
UPPER_YELLOW_HSV = [45, 255, 255]  # Yellow
LOWER_HSV_ARRAY = [
    LOWER_RED_HSV, LOWER_BLUE_HSV, LOWER_GREEN_HSV, LOWER_YELLOW_HSV
]
UPPER_HSV_ARRAY = [
    UPPER_RED_HSV, UPPER_BLUE_HSV, UPPER_GREEN_HSV, UPPER_YELLOW_HSV
]

# Define the colour boundaries in YUV
LOWER_RED_YUV = [30, 100, 133]  # Red
UPPER_RED_YUV = [255, 127, 255]  # Red
LOWER_BLUE_YUV = [36, 122, 49]  # Blue
UPPER_BLUE_YUV = [255, 255, 123]  # Blue
LOWER_GREEN_YUV = [31, 108, 0]  # Green
UPPER_GREEN_YUV = [255, 132, 123]  # Green
LOWER_YELLOW_YUV = [57, 0, 117]  # Yellow
UPPER_YELLOW_YUV = [255, 132, 185]  # Yellow
LOWER_YUV_ARRAY = [
    LOWER_RED_YUV, LOWER_BLUE_YUV, LOWER_GREEN_YUV, LOWER_YELLOW_YUV
]
UPPER_YUV_ARRAY = [
    UPPER_RED_YUV, UPPER_BLUE_YUV, UPPER_GREEN_YUV, UPPER_YELLOW_YUV
]

# Initialize colour array counter
COLOUR_NAME_ARRAY = ['Red', 'Blue', 'Green', 'Yellow']
