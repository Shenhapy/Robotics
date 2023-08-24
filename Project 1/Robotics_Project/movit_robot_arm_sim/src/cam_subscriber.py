#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(data):
 
    try:
        # Convert ROS Image message to OpenCV format
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Camera Image", cv_image)
        
        cv2.waitKey(1)  # Refresh display (1 millisecond delay)
    except Exception as e:
        rospy.logerr(e)

def image_subscriber():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber('/camera_model/camera/image_topic', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass

