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
import math
import rospy
import numpy as np

from pathlib import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from human_aware_robot_navigation.srv import *
from human_aware_robot_navigation.msg import *

# def printDetails(self, image):
#     print("Height: ", image.height)
#     print("Width: ", image.width)
#     print("Encoding: ", image.encoding)
#     print("Step: ", image.step)

# # Raw to OpenCV conversion
# def store(self, cv_image):
#     """
#         Stores the converted cv_image
#         in memory to be further processed
#         by a different node.
#
#         Arguments:
#             param1: MAT image
#     """
#     cv2.imwrite(self.path + "/data/depth_image/depth_image.png", cv_image)

def toMat(depth_raw_image):
    """
        Converts depth image into
        a floating point numpy array
        of meter values.
    """
    try:
        # self.printDetails(depth_raw_image)

        # Depth raw image to OpenCV format
        depth_image = CvBridge().imgmsg_to_cv2(depth_raw_image, '32FC1')
        # cv2.normalize(depth_image, depth_image, 0, 1, cv2.NORM_MINMAX)
        # print("Converted: ", depth_image)

        # Save greyscale image to memore
        # N.B: This is why we multiply by 255
        # self.store(depth_image**255)

        self.setDepthImage(depth_image)

    except Exception as CvBridgeError:
        print('Error during image conversion: ', CvBridgeError)

def getDepths(msg):
    """
        Receives detections and
        fetches their respective
        distances from the depth
        map, building a Distances
        message.

        Arguments:
            param1: Detections

        Returns:
            Distances: Distances message
    """
    # Custom distances message
    distances = Distances()

    print("Received message: depth")

    # # Access depth image
    # depth_image = self.getDepthImage()
    #
    # if msg.req.number_of_detections > 0:
    #     for detection in msg.req.detections:
    #         # print(detection)
    #         for detail in detection.details:
    #             print("Detail: ", detail)
    #             distance = Distance()
    #             distance.ID = detail.ID
    #             distance.rgb_x = detail.rgb_x
    #             distance.rgb_y = detail.rgb_y
    #             distance.distance = depth_image[detail.rgb_x, detail.rgb_x] if not math.isnan(depth_image[detail.rgb_x, detail.rgb_x]) else None
    #             distances.distances.append(distance)

    return RequestDepthResponse(distances)

def main(args):

    # Initialise node
    rospy.init_node('mapping', anonymous=True)

    try:
        # Detection service
        service = rospy.Service('rgb_to_depth_mapping', RequestDepth, getDepths)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
