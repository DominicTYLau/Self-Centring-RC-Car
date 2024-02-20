import cv2
import numpy as np
from picamera2 import Picamera2
import math
import smbus
import time

bus = smbus.SMBus(1)
address = 0x08


# Initialize the camera
piCam = Picamera2()
piCam.preview_configuration.main.size = (320, 240)
piCam.preview_configuration.main.format = "YUV420"
piCam.set_controls({"AwbEnable": 0, "AeEnable": 0})
piCam.preview_configuration.align()
piCam.configure("preview")
piCam.start()

def writeData(value):
    byteValue = StringToBytes(value)
    try:
        bus.write_i2c_block_data(address, 0x00, byteValue)
    except OSError as e:
        print(f"I2C Error: {e}")
    return -1


def StringToBytes(val):
        retVal = []
        for c in val:
                retVal.append(ord(c))
        return retVal

def process_image(image):
    gauss_image = cv2.GaussianBlur(image, (5,5),0)
    canny_image = cv2.Canny(gauss_image, 50, 150)

    # Define the region of interest
    height, width = image.shape
    polygons = np.array([[(0, height), (0, 270), (width, 270), (width, height)]], dtype=np.int32)
    mask = np.zeros_like(image)
    cv2.fillPoly(mask, polygons, 255)
    return cv2.bitwise_and(canny_image, mask)

def displayLines(image, lines, colour=[255, 0, 0], thickness=3):
    # Display lines on the image
    lineImage = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(lineImage, (int(x1), int(y1)), (int(x2), int(y2)), colour, thickness)
    return lineImage

while True:
    # Capture frame-by-frame
    frame = np.empty((240, 320, 1), dtype=np.uint8)
    frame = cv2.flip(piCam.capture_array(), -1)
    
    # Process the image
    lane_image = np.copy(frame)
    cropped_image = process_image(lane_image)
    
    # Detect lines using Hough transform
    lines = cv2.HoughLinesP(cropped_image, 1, np.pi/180, 50, np.array([]), minLineLength=10, maxLineGap=3)

    if lines is not None:
        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1)
                if np.abs(slope) < 0.5:
                    continue
                if slope <= 0:
                    left_line_x.extend([x1, x2])
                    left_line_y.extend([y1, y2])
                else:
                    right_line_x.extend([x1, x2])
                    right_line_y.extend([y1, y2])

        min_y = lane_image.shape[0] * (3/5) # <-- Just below the horizon
        max_y = lane_image.shape[0] # <-- The bottom of the image

        if left_line_y and left_line_x:  # Check if left_line_y and left_line_x are not empty
            poly_left = np.poly1d(np.polyfit(left_line_y, left_line_x, deg=1))
            left_x_start = int(poly_left(max_y))
            left_x_end = int(poly_left(min_y))
        else:
            left_x_start = left_x_end = 0

        if right_line_y and right_line_x:  # Check if right_line_y and right_line_x are not empty
            poly_right = np.poly1d(np.polyfit(right_line_y, right_line_x, deg=1))
            right_x_start = int(poly_right(max_y))
            right_x_end = int(poly_right(min_y))            
        else:
            
            right_x_start = right_x_end = lane_image.shape[1]
        
        line_image = displayLines(
            lane_image,
            [[
                [left_x_start, max_y, left_x_end, min_y],
                [right_x_start, max_y, right_x_end, min_y],
            ]],
            thickness = 10,
        )
        
        if left_x_start != left_x_end and right_x_start != right_x_end:
            left_slope = (min_y - max_y) / (left_x_end - left_x_start)
            right_slope = (min_y - max_y) / (right_x_end - right_x_start)
            
            left_intercept = min_y - left_slope * left_x_end
            right_intercept = min_y - right_slope * right_x_end
            
            if left_slope - right_slope != 0:
                intersection_x = (right_intercept - left_intercept) / (left_slope - right_slope)
                intersection_y = left_slope * intersection_x + left_intercept
                
                intersection_point = (int(intersection_x), int(intersection_y))
                print("Intersection point:", intersection_point)
                writeData(str(int(intersection_x)))
                cv2.circle(line_image, intersection_point, 5, (0, 255, 0), -1) # Draw intersection point
        else:

            mid_x_start = int((left_x_start + right_x_start)/2)
            mid_x_end = int((left_x_end + right_x_end)/2)
            
            mid_y = 310
            
            if (min_y - max_y) != 0:
                mid_slope = (mid_x_end - mid_x_start) / (max_y - min_y)
                print(mid_slope)
            else:
                mid_slope = 0

            b = min_y - mid_slope * mid_x_end
                  
        # Check if the denominator is zero before performing the division
            if mid_slope != 0:
                x = int((mid_y - b) / mid_slope)
                print(x)
                writeData(str(x))
            else:
                print("160")
                writeData("160")

        # Combine the original image with the line image
        combo_image = cv2.addWeighted(lane_image, 0.8, line_image, 1, 0)

        # Display the resulting image
        cv2.imshow("roi", cropped_image)
        cv2.imshow("new", combo_image)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) == ord('q'):
        break

piCam.stop_preview()
cv2.destroyAllWindows()
