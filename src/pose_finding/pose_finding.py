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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

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

        # Poses, distance and marker publishers
        self.poses_pub = rospy.Publisher('poses', Poses, queue_size=5)
        self.distance_pub = rospy.Publisher('distances', Distances, queue_size=5)
        self.markers_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)

        # Camera info subscriber
        self.info_sub  = rospy.Subscriber("/xtion/rgb/camera_info", CameraInfo, self.setCameraInfo)

    def setCameraInfo(self, info_msg):
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

    def getRegionOfInterest(self, i, j, d, cv_depth_image):
        """
            Returns the distances around the
            center point of the bounding box,
            a.k.a our ROI, which is used to
            find the average distance. The ROI
            is filtered such that non valid values
            are removed (such as NaN).

            Arguments:
                int: i is the centre_x point of the bounding box centre
                int: j is the centre_y point of the bounding box centre
                int: d is the size of the neighbourhood around the centre
                MAT: Depth image in CV MAT format

            Returns:
                Numpy array: Array with distance values
        """
        # Fetch all distances around the centre
        # point (d away from the centre point)
        roi = cv_depth_image[j-d:j+d+1, i-d:i+d+1]

        # Return all the valid values
        # within the ROI (all NaN values are removed)
        return roi[~np.isnan(roi)]

    def createDistance(self, ID, average_distance):
        """
            Creates a distance message object
            for the detection.

            Arguments:
                int: The ID of the detection
                float: The average distance from the detection (in meters)

            Returns:
                Distance: Distance object message
        """
        # Create distance message for the
        # single detection. Note that the
        # distance is averaged
        distance = Distance()
        distance.ID = ID
        distance.distance = average_distance
        return distance

    def createPointStamped(self, i, j, average_distance):
        """
            Creates a point stamped object for
            the real world pose transformation.

            Arguments:
                int: i is the centre_x point of the bounding box centre
                int: j is the centre_y point of the bounding box centre
                float: The average distance from the detection (in meters)

            Returns:
                PointStamped: PointStamped detection message
        """
        # Backproject the pixel point in the optical frame
        phm_point = self.pcm.projectPixelTo3dRay((j,i))

        # Create a Point message with
        # the backprojection data and
        # the retrieved depth data
        point = Point()
        point.x = phm_point[0]
        point.y = phm_point[1]
        point.z = average_distance

        # Create a custom header for the
        # point object (necessary for the map
        # transform)
        transformHeader = Header()
        transformHeader.stamp = rospy.Time(0)
        transformHeader.frame_id = "xtion_rgb_optical_frame"

        # Create pointStamped object
        # for the real world spatial
        # transformation and combine it
        # with the previous Point message
        # and transformHeader
        pointStamped = PointStamped()
        pointStamped.header = transformHeader
        pointStamped.point = point

        return pointStamped

    def createPose(self, ID, transformed):
        """
            Creates a geometry_msgs point object
            containing detection's real world map
            pose.

            Arguments:
                int: The detection ID
                geometry_msgs/PointStamped: The map pose of the detection

            Returns:
                geometry_msgs/Pose: Detection pose in the map
        """
        # Create Pose message for detection
        pose = Pose()
        pose.ID = ID
        pose.pose = transformed.point

        return pose

    def createMarker(self, ID, transformed):
        """
            Creates a marker visual message
            for the detection

            Arguments:
                int: The detection ID
                geometry_msgs/PointStampe: The map pose of the detection

            Returns:
                visualization_msgs/Marker : Detection marker
        """
        # Create Marker and add
        # meta-information
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "harn"
        marker.id = ID
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        # Marker pose
        marker.pose.position.x = transformed.point.x
        marker.pose.position.y = transformed.point.y
        marker.pose.position.z = transformed.point.z

        # Marker orientation
        # (we don't care about this, at the moement)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0

        # Marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # Marker lifetime
        marker.lifetime.secs = 7

        return marker

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
        # Message objects
        poses = Poses()
        distances = Distances()
        markerArray = MarkerArray()

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

                # Get region of interest around centre point
                roi = self.getRegionOfInterest(i, j, d, cv_depth_image)

                # Compute average distance over neighbours
                average_distance = np.sum(roi) / roi.size

                # Add detection distance to the distance array
                distances.array.append(self.createDistance(detection.ID, average_distance))

                # # if not math.isnan(cv_depth_image[j,i]):
                # phm_point = self.pcm.projectPixelTo3dRay((j,i))
                # # a = 0.00173667
                # # x_world = (j-320)*a*cv_depth_image[j,i]
                # # y_world = (i-240)*a*cv_depth_image[j,i]
                # # print("3D pointb: ", x_world, y_world, cv_depth_image[j,i])
                #
                # # Create geometry_msgs
                # # point = Point()
                # # point.x = x_world
                # # point.y = y_world
                # # point.z = cv_depth_image[j,i]
                # # print("Point: ", point)
                #
                # # Create a Point message
                # # for real world map transform
                # point = Point()
                # point.x = phm_point[0]
                # point.y = phm_point[1]
                # point.z = average_distance
                #
                # # Create a custom header
                # # for the real world spatial
                # # transformation
                # transformHeader = Header()
                # transformHeader.stamp = rospy.Time(0)
                # transformHeader.frame_id = "xtion_rgb_optical_frame"
                #
                # # Create pointStamped object
                # # for the real world spatial
                # # transformation
                # pointStamped = PointStamped()
                # pointStamped.header = transformHeader
                # pointStamped.point = point

                # Get transform stamped point
                pointStamped = self.createPointStamped(i, j, average_distance)

                try:
                    # Transform optical frame point onto map coordinate
                    transformed = self.tf_listener.transformPoint("map", pointStamped)

                    # Aggregate pose to poses array
                    poses.array.append(self.createPose(detection.ID, transformed))

                    # Aggregate marker with markers array
                    markerArray.markers.append(self.createMarker(detection.ID, transformed))

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    print(e)

        # Publish distances and poses
        self.poses_pub.publish(poses)
        self.distance_pub.publish(distances)
        self.markers_pub.publish(markerArray)

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
