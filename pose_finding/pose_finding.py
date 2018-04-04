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
        # Publishing rate
        self.rate = rospy.Rate(10)

        # Set camera info boolean flag
        self.cameraInfoInitialised = False

        # PinholeCameraModel object
        self.pcm = PinholeCameraModel()

        # Tf listener
        self.tf_listener = tf.TransformListener()

        # Constant path
        self.path = str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0])

        # Poses and distance publishers
        self.poses_pub = rospy.Publisher('poses', Poses, queue_size=5)
        self.distance_pub = rospy.Publisher('distances', Distances, queue_size=5)

        # Camera info subscriber
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
        # Custom distances and poses
        # message objects
        poses = Poses()
        distances = Distances()

        # Convert depth image into MAT
        cv_depth_image = self.toMat(req.depth)

        # Populate our distances array
        if req.detections.number_of_detections > 0:

            # Iterate over the detections
            for detection in req.detections.array:

                # Size of neighbours around
                # the centre point to take in
                # consideration for the distance
                # average
                d = 30

                # Centre point of the detection
                # box got from the service caller
                i = detection.centre_x
                j = detection.centre_y

                # Fetch all distances around the centre
                # point (d away from the centre point),
                # and remove all those that evaluate to NaN
                roi = cv_depth_image[j-d:j+d+1, i-d:i+d+1]
                roi = roi[~np.isnan(roi)]

                # Compute average distance
                # over centre neighbours
                average_distance = np.sum(roi) / roi.size

                # Create distance message for the
                # single detection. Note that the
                # distance is averaged
                distance = Distance()
                distance.ID = detection.ID
                distance.distance = average_distance
                distances.array.append(distance)

                # if not math.isnan(cv_depth_image[j,i]):
                phm_point = self.pcm.projectPixelTo3dRay((j,i))
                # a = 0.00173667
                # x_world = (j-320)*a*cv_depth_image[j,i]
                # y_world = (i-240)*a*cv_depth_image[j,i]
                # print("3D pointb: ", x_world, y_world, cv_depth_image[j,i])

                # Create geometry_msgs
                # point = Point()
                # point.x = x_world
                # point.y = y_world
                # point.z = cv_depth_image[j,i]
                # print("Point: ", point)

                # Create a Point message
                # for real world map transform
                point = Point()
                point.x = phm_point[0]
                point.y = phm_point[1]
                point.z = average_distance

                # Create a custom header
                # for the real world spatial
                # transformation
                transformHeader = Header()
                transformHeader.stamp = rospy.Time(0)
                transformHeader.frame_id = "xtion_rgb_optical_frame"

                # Create pointStamped object
                # for the real world spatial
                # transformation
                pointStamped = PointStamped()
                pointStamped.header = transformHeader
                pointStamped.point = point

                try:
                    # Transform optical frame point onto map coordinate
                    transformed = self.tf_listener.transformPoint("map", pointStamped)
                    print("Trans: ", transformed)

                    # Create Pose message for detection
                    pose = Pose()
                    pose.ID = detection.ID
                    pose.pose = transformed.point

                    # Aggregate pose with the remaining ones
                    poses.array.append(pose)

                    print("Pose: ", pose)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(e)

        # Publish distances and poses
        self.poses_pub.publish(poses)
        self.distance_pub.publish(distances)

        # Return response back to the caller
        return RequestDepthResponse("success")

def main(args):

    # Initialise node
    rospy.init_node('mapping', anonymous=True)

    try:
        # Create class object
        pf = PoseFinding()

        # Detection service
        service = rospy.Service('detection_pose', RequestDepth, pf.getPoses)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
