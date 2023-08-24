#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int32

def detect_polygon_color(image):
 


    # Red 
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Green 
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    # Lemon Green 
    lower_lemon_green = np.array([25, 40, 40])
    upper_lemon_green = np.array([45, 255, 255])

    # Blue 
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])

    # Light Blue 
    lower_light_blue = np.array([80, 40, 40])
    upper_light_blue = np.array([100, 255, 255])

    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Create masks for each color range
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_lemon_green = cv2.inRange(hsv, lower_lemon_green, upper_lemon_green)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_light_blue = cv2.inRange(hsv, lower_light_blue, upper_light_blue)

    # Combine masks to detect all ranges of red, green, lemon green, blue, and light blue
    mask_red = mask_red1 | mask_red2
    mask_green_or_lemon_green = mask_green | mask_lemon_green
    mask_blue_or_light_blue = mask_blue | mask_light_blue

    # Check if any box is detected
    if mask_red.any():
        return 1  # Red
    elif mask_green_or_lemon_green.any():
        return 2  # Green or Lemon Green
    elif mask_blue_or_light_blue.any():
        return 3  # Blue or Light Blue
    else:
        return 0  # None

def image_callback(msg):
    # Convert ROS image message to OpenCV image
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Detect the polygon color
    color_detected = detect_polygon_color(cv_image)

    # Publish the result as an integer on the output topic
    color_pub.publish(color_detected)

if __name__ == "__main__":
    rospy.init_node("color_detection_node")

    # Create a subscriber for the input image topic
    image_sub = rospy.Subscriber("/camera_model/camera/image_topic", Image, image_callback)

    # Create a publisher for the output color topic
    color_pub = rospy.Publisher("output_color_topic", Int32, queue_size=10)

    rospy.spin()

