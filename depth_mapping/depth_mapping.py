#!/usr/bin/env python

"""
    The following script converts
    the ROS std_msgs/Image type to
    the OpenCV MAT format.
"""

# Modules
import os
import cv2
import sys
import rospy

# Path module
from pathlib import Path

# Import ROS Image data type
from sensor_msgs.msg import Image

# Import cv_bridge (ROS interface for conversion)
from cv_bridge import CvBridge, CvBridgeError

# Custom service message
from human_aware_robot_navigation.srv import *

def requestDetection(req):
    """
        Requests detection
        to service module.

        Arguments:
            param1: request string

        Returns:
            int: Service response (positive, negative)
    """
    # Wait for service to come alive
    rospy.wait_for_service('detection')

    try:
        # Build request
        request = rospy.ServiceProxy('detection', RequestDetection)

        # Get response from service
        res = request(req)

        # Response
        rospy.loginfo("Detection service: %s", res.response)
        return res.response

    except Exception as e:
        raise

# Raw to OpenCV conversion
def store(cv_image):
    """
        Stores the converted cv_image
        in memory to be further processed
        by a different node.

        Arguments:
            param1: MAT image
    """
    cv2.imwrite(str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0]) + "/data/converted/image.jpg", cv_image)

# Raw to OpenCV conversion
def toMAT(raw_image):
    """
        Converts raw RGB image
        into MAT format.

        Arguments:
            param1: raw RGB image
    """
    try:
        # RGB raw image to OpenCV bgr MAT format
        cv_image = CvBridge().imgmsg_to_cv2(raw_image, 'bgr8')

        # Store image
        store(cv_image)

        # Send detection request to service
        rospy.loginfo("Requesting detection service on converted image...")
        requestDetection("request")

    except Exception as CvBridgeError:
        print('Error during image conversion: ', CvBridgeError)

def main(args):

    # Initialise node
    rospy.init_node('conversion', anonymous=True)

    try:
        # Subscribe to TIAGo's image_raw topic
        image_raw = rospy.Subscriber('xtion/rgb/image_raw', Image, toMAT)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
