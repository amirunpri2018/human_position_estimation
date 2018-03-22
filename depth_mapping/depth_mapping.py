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
np.set_printoptions(threshold='nan')

from pathlib import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from human_aware_robot_navigation.srv import *
from human_aware_robot_navigation.msg import *

# # Raw to OpenCV conversion
def store(cv_image):
    """
        Stores the converted cv_image
        in memory to be further processed
        by a different node.

        Arguments:
            param1: MAT image
    """
    cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
    cv2.imwrite(str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0]) + "/data/depth_image/depth_image.png", cv_image*255)

def toMat(depth_raw_image):
    """
        Converts depth image into
        a floating point numpy array
        of meter values.
    """
    try:
        # Depth raw image to OpenCV format
        cv_depth_image = CvBridge().imgmsg_to_cv2(depth_raw_image, '32FC1')

        # DO NOT uncomment
        # store(cv_depth_image)

        return cv_depth_image

    except Exception as CvBridgeError:
        print('Error during image conversion: ', CvBridgeError)

def getDepths(req):
    """
        Receives detections and
        fetches their respective
        distances from the depth
        map, building a Distances
        message.

        Arguments:
            sensor_msgs/Image: Raw depth image

        Returns:
            Distances: Distances message
    """
    # Custom distances message
    distances = Distances()

    # Convert depth image into MAT
    cv_depth_image = toMat(req.depth)

    # Populate our distances array
    if req.detections.number_of_detections > 0:

        # Iterate over the detections
        for detection in req.detections.array:

            # Sliding window centre point
            # and size of neighbours
            d = 30
            i = detection.centre_x
            j = detection.centre_y

            print("Centre point: ", cv_depth_image[j,i])

            # Fetch window around the centre of the
            # bounding box and get rid of NaN values
            roi = cv_depth_image[j-d:j+d+1, i-d:i+d+1]
            roi = roi[~np.isnan(roi)]

            # print("ROI: ", roi)

            # Copy over detections items
            # for future 1 to 1 mapping
            distance = Distance()
            distance.ID = detection.ID

            # Compute average distance over
            # ROI window
            avr_distance = np.sum(roi) / roi.size

            # Average distance of the person
            distance.distance = avr_distance

            # Aggregate the distance to the
            # general distances array
            distances.array.append(distance)

    # Return response back to the caller
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
