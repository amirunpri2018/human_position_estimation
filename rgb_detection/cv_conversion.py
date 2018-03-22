#!/usr/bin/env python

"""
    The following script converts
    the ROS std_msgs/Image type to
    OpenCV MAT format.
"""

# Modules
import os
import cv2
import sys
import rospy
import math
import message_filters

# Path library
from pathlib import Path

# OpenCV-ROS bridge
from cv_bridge import CvBridge, CvBridgeError

# Messages for requests and subscriptions
from sensor_msgs.msg import Image, PointCloud2
from human_aware_robot_navigation.srv import *

# Constant path
PATH = str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0])

def requestDetection(req):
    """
        Sends a service request to
        the person detection module.

        Arguments:
            sensor_msgs/Image: Depth image

        Returns:
            string: Service response
    """
    # Wait for service to come alive
    rospy.wait_for_service('detection')

    try:
        # Build request
        request = rospy.ServiceProxy('detection', RequestDetection)

        # Get response from service
        response = request(req)

        # Access the response field of the custom msg
        rospy.loginfo("Detection service: %s", response.res)
        # return response.res

    except rospy.ServiceException as e:
        rospy.loginfo("Detection service call failed: %s", e)

def store(cv_image):
    """
        Stores the converted raw image from
        the subscription and writes it to
        memory.

        Arguments:
            MAT: OpenCV formatted image
    """
    cv2.imwrite(PATH + "/data/converted/image.png", cv_image)

def toMAT(rgb_image):
    """
        Converts raw RGB image
        into OpenCV's MAT format.

        Arguments:
            sensor_msgs/Image: RGB raw image

        Returns:
            MAT: OpenCV BGR8 MAT format
    """
    try:
        cv_image = CvBridge().imgmsg_to_cv2(rgb_image, 'bgr8')
        return cv_image

    except Exception as CvBridgeError:
        print('Error during image conversion: ', CvBridgeError)

def processSubscriptions(rgb_image, depth_image):
    """
        Callback for the TimeSynchronizer
        that receives both the rgb raw image
        and the depth image, respectively
        running the detection module on the
        former and the mappin process on the
        former at a later stage in the chain.

        Arguments:
            sensor_msgs/Image: The RGB raw image
            sensor_msgs/Image: The depth image
    """
    print("Got depth and rgb.")
    # Processing the rgb image
    rgb_cv_image = toMAT(rgb_image)
    store(rgb_cv_image)

    # Request services
    rospy.loginfo("Requesting detection and mapping services")
    requestDetection(depth_image)

def main(args):

    # Initialise node
    rospy.init_node('subscriptions_sync', anonymous=True)

    try:
        # Subscriptions (via Subscriber package)
        rgb_sub   = message_filters.Subscriber("/xtion/rgb/image_rect_color", Image)
        depth_sub = message_filters.Subscriber("/xtion/depth_registered/hw_registered/image_rect_raw", Image)
        # depth_sub = message_filters.Subscriber("/xtion/depth_registered/hw_registered/image_rect_raw", Image)
        # point_sub = message_filters.Subscriber("/xtion/depth_registered/points", PointCloud2)

        # Synchronize subscriptions
        ats = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=2, slop=0.3)
        ats.registerCallback(processSubscriptions)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
