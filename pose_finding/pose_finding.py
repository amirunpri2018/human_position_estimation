#!/usr/bin/env python

"""
    Pose Finding Service.

    ROS service that estimates
    the real world coordinate
    pose of the detected humans.
"""

# Modules
import os
import tf
import cv2
import sys
import math
import rospy
import numpy as np

from pathlib import Path
from point_cloud2 import *
from cv_bridge import CvBridge, CvBridgeError
from human_aware_robot_navigation.srv import *
from human_aware_robot_navigation.msg import *
from image_geometry import PinholeCameraModel
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, PointStamped

class PoseFinding:

    def __init__(self):
        """
            Constructor.
        """
        # Tf listener
        self.tf_listener = tf.TransformListener()

        # Set camera info boolean flag
        self.cameraInfoInitialised = False

        # PinholeCameraModel object
        self.pcm = PinholeCameraModel()

        # Publishing rate
        self.rate = rospy.Rate(10)

        # Constant path
        self.path = str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0])

        # Publisher (custom detection message)
        self.info_sub  = rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, self.cameraInfoCallback)

    def cameraInfoCallback(self, info_msg):
        """
            Sets the received info msg.

            Arguments:
                sensor_msgs/Image: Camera info msg
        """
        if not self.cameraInfoInitialised:
            self.pcm.fromCameraInfo(info_msg)
            self.cameraInfoInitialised = True

    # Raw to OpenCV conversion
    def store(self, cv_image):
        """
            Stores the converted cv_image
            in memory to be further processed
            by a different node.

            Arguments:
                MAT: OpenCV MAT format image
        """
        cv2.normalize(cv_image, cv_image, 0, 1, cv2.NORM_MINMAX)
        cv2.imwrite(self.path + "/data/depth_image/depth_image.png", cv_image*255)

    def toMat(self, depth_raw_image):
        """
            Converts depth image into
            a floating point numpy array
            of meter values.
        """
        try:
            # Depth raw image to OpenCV format
            cv_depth_image = CvBridge().imgmsg_to_cv2(depth_raw_image, '32FC1')

            # DO NOT uncomment
            # self.store(cv_depth_image)

            return cv_depth_image

        except Exception as CvBridgeError:
            print('Error during image conversion: ', CvBridgeError)

    def getPoses(self, req):
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
        cv_depth_image = self.toMat(req.depth)

        # Populate our distances array
        if req.detections.number_of_detections > 0:

            # Iterate over the detections
            for detection in req.detections.array:

                # Sliding window centre point
                # and size of neighbours
                d = 30
                i = detection.centre_x
                j = detection.centre_y

                if not math.isnan(cv_depth_image[j,i]):
                    # print("Centre point: ", cv_depth_image[j,i])
                    # print("Rectify point: ", self.pcm.rectifyPoint((j,i)))
                    phm_point = self.pcm.projectPixelTo3dRay((j,i))
                    # print("2D point: ", self.pcm.project3dToPixel((2.19, -1.34, 0.391)))
                    # print("P matrix: ", self.pcm.projectionMatrix())
                    a = 0.00173667
                    x_world = (j-320)*a*cv_depth_image[j,i]
                    y_world = (i-240)*a*cv_depth_image[j,i]
                    # print("3D pointb: ", x_world, y_world, cv_depth_image[j,i])

                    # Create geometry_msgs
                    # point = Point()
                    # point.x = x_world
                    # point.y = y_world
                    # point.z = cv_depth_image[j,i]
                    # print("Point: ", point)

                    point = Point()
                    point.x = phm_point[0]
                    point.y = phm_point[1]
                    point.z = cv_depth_image[j,i]

                    # Header message
                    h = Header()
                    h.stamp = rospy.Time(0)
                    h.frame_id = "xtion_rgb_optical_frame"

                    pointStamped = PointStamped()
                    pointStamped.header = h
                    pointStamped.point = point
                    # print("PointStamped: ", pointStamped)

                    try:
                        transformed = self.tf_listener.transformPoint("map", pointStamped)
                        print("Transformed: ", transformed.point)
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        pass

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

                cv_depth_image[j-d:j+d+1, i-d:i+d+1] = 8
                self.store(cv_depth_image)

        # Return response back to the caller
        return RequestDepthResponse(distances)

def main(args):

    # Initialise node
    rospy.init_node('mapping', anonymous=True)

    try:
        # Create class object
        pf = PoseFinding()

        # Detection service
        service = rospy.Service('rgb_to_depth_mapping', RequestDepth, pf.getPoses)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
