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
        store(cv_depth_image)

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

    # Draw all valid points in the depth map
    for x in range(cv_depth_image.shape[0]):
        for y in range(cv_depth_image.shape[1]):
            if not math.isnan(cv_depth_image[x,y]) and cv_depth_image[x,y] > 0:
                cv2.circle(cv_depth_image, (x, y), 4, (255,255,255), -1)

    cv2.imwrite(str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0]) + "/data/depth_image/depth_image_valid.png", cv_depth_image)

    # # Populate our distances array
    if req.detections.number_of_detections > 0:

        # Iterate over the detections
        for detection in req.detections.array:

            # ROI cv_depth_image (around person)
            roi_depth_image = cv_depth_image[detection.top_left_y:detection.top_left_y + detection.height,
                                             detection.top_left_x:detection.top_left_x + detection.width]

            # Copy over detections items
            # for future 1 to 1 mapping
            distance = Distance()
            distance.ID = detection.ID

            # Convert all nan to zeros
            roi_depth_image = np.nan_to_num(roi_depth_image)
            print("ROI shape: ", roi_depth_image.shape)

            # Compute average distance in the
            # bounding box
            roi_sum = np.nansum(roi_depth_image)

            print("Sum: ", roi_sum)
            print("Size: ", roi_depth_image.size)

            # Average distance of the person
            distance.distance = roi_sum / roi_depth_image.size

            print("Distance: " + str(roi_sum / roi_depth_image.size) + "\n")

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
