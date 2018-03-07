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
import numpy as np

from pathlib import Path
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from human_aware_robot_navigation.srv import *
from human_aware_robot_navigation.msg import *

class DepthRetrieval:

    def __init__(self):
        """
            Constructor.
        """
        # Global converted depth_image
        self.depth_image = None

        # Custom distances message
        self.msg_distances = Distances()

        # Constant path
        self.path = str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0])

        # Depth image subscriber
        self.depth_sub = rospy.Subscriber('/xtion/depth/image_raw', Image, self.convert)

    # Raw to OpenCV conversion
    def store(self, cv_image):
        """
            Stores the converted cv_image
            in memory to be further processed
            by a different node.

            Arguments:
                param1: MAT image
        """
        cv2.imwrite(self.path + "/data/depth_image/depth_image.jpg", cv_image)

    def convert(self, depth_raw_image):
        try:
            # Depth raw image to OpenCV format
            depth_image = CvBridge().imgmsg_to_cv2(depth_raw_image, 'passthrough')

            # Save greyscale image to memore
            # N.B: This is why we multiply by 255
            self.store(depth_image * 255)

            self.setDepthImage(depth_image)

        except Exception as CvBridgeError:
            print('Error during image conversion: ', CvBridgeError)

    def setDepthImage(self, depth_image):
        self.depth_image = depth_image

    def getDepthImage(self):
        return self.depth_image

    # Raw to OpenCV conversion
    def getDepths(self, msg):
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
        # Access details about detection in the msg
        print(msg.detections.detections)
        # if msg.detections.detections:
        #     for detail in msg.detections.detections.details:
        #         # Create distance message
        #         # and populate it with depth
        #         # information
        #         distance = Distance()
        #         distance.ID = detail.ID
        #         distance.rgb_x = detail.rgb_x
        #         distance.rgb_y = detail.rgb_y
        #         distance.distance = self.getDepthImage()[detail.rgb_x, detail.rgb_y]
        #
        #         self.msg_distances.distances.append(distance)
        #
        return RequestDepthResponse(self.msg_distances)

def main(args):

    # Initialise node
    rospy.init_node('depths_server', anonymous=True)

    try:
        # Initialise
        dr = DepthRetrieval()

        rospy.loginfo("Warm-up data...")
        rospy.sleep(3)

        # Detection service
        service = rospy.Service('detections_distances', RequestDepth, dr.getDepths)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
