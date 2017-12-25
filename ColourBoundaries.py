# Define the colour boundaries in HSV
LOWER_RED_LFT_HSV = [165, 50, 50]  # Left of 0deg Red = ~330deg to 359deg
UPPER_RED_LFT_HSV = [179, 255, 255]  # Red
LOWER_RED_HSV = [0, 50, 50]  # Red = 0deg to ~30deg
UPPER_RED_HSV = [15, 255, 255]  # Red
LOWER_BLUE_HSV = [80, 50, 50]  # Blue = ~180deg to ~260deg
UPPER_BLUE_HSV = [140, 255, 255]  # Blue
LOWER_GREEN_HSV = [45, 50, 50]  # Green = ~90deg to ~150deg
UPPER_GREEN_HSV = [75, 255, 255]  # Green
LOWER_YELLOW_HSV = [20, 50, 50]  # Yellow = ~40deg to ~90deg
UPPER_YELLOW_HSV = [45, 255, 255]  # Yellow
LOWER_HSV_ARRAY = [
    LOWER_RED_HSV, LOWER_BLUE_HSV, LOWER_GREEN_HSV, LOWER_YELLOW_HSV
]
UPPER_HSV_ARRAY = [
    UPPER_RED_HSV, UPPER_BLUE_HSV, UPPER_GREEN_HSV, UPPER_YELLOW_HSV
]

# Initialize colour array counter
COLOUR_NAME_ARRAY = ['Red', 'Blue', 'Green', 'Yellow']
