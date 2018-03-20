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

        # Store depth image
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

            # ROI cv_depth_image (around person)
            roi = cv_depth_image[detection.top_left_y:detection.top_left_y + detection.height,
                                 detection.top_left_x:detection.top_left_x + detection.width]

            i = detection.centre_x
            j = detection.centre_y
            d = 15

            # Centre neighbours
            neighbours = cv_depth_image[i-d:i+d+1, j-d:j+d+1].flatten()
            neighbours = neighbours[~np.isnan(neighbours)]
            print(neighbours[~np.isnan(neighbours)])

            # Filter depth image from invalid values (NaN)
            roi = roi[~np.isnan(roi)]
            print("ROI: ", roi[0].x)
            # print(roi[~np.isnan(roi)])

            # Copy over detections items
            # for future 1 to 1 mapping
            distance = Distance()
            distance.ID = detection.ID

            # Compute average distance
            roi_sum = np.sum(roi)
            # roi_sum = np.sum(neighbours)

            # Average distance of the person
            distance.distance = roi_sum / roi.size
            # distance.distance = roi_sum / neighbours.size

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
