#!/usr/bin/env python

"""
    Human Detection Service.

    ROS service that takes as input
    an image in a MAT format, and
    returns a frame with detections
    and their respective centre points.
"""

# Modules
import os
import cv2
import sys
import time
import math
import rospy
import imutils
import numpy as np

from pathlib import Path
from sensor_msgs.msg import Image
from human_aware_robot_navigation.msg import *
from human_aware_robot_navigation.srv import *

class PersonDetection:

    def __init__(self):
        """
            Constructor.
        """
        # Detection target ID (person)
        self.target = 15

        # Minimum confidence for acceptable detections
        self.confidence = 0.5

        # Converted depth_image
        self.depth_image = None

        # Publishing rate
        self.rate = rospy.Rate(10)

        # Number of detections
        self.number_of_detections = 0

        # Detection messages
        self.detections = Detections()

        # Constant path
        self.path = str(Path(os.path.dirname(os.path.abspath(__file__))).parents[0])

        # Define detection's target/s
        self.targets = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

        # Bounding boxes self.colours
        self.colours = np.random.uniform(0, 255, size=(len(self.targets), 3))

        # Load NN's serialised model
        self.net = cv2.dnn.readNetFromCaffe(self.path + "/data/nn_params/MobileNetSSD_deploy.prototxt.txt",
                                            self.path + "/data/nn_params/MobileNetSSD_deploy.caffemodel")

        # Publisher (custom detection message)
        self.detection_pub = rospy.Publisher('detections', Detections, queue_size=5)

        # Publisher (custom detection message)
        self.depth_pub = rospy.Publisher('distances', Distances, queue_size=5)

        print("[INFO] Successful Initialisation")

    def detection(self, req):
        """
            Returns the frame with
            detections bounding boxes.

            Params:
                sensor_msgs/Image: Depth image syncd with RGB

            Ouput:
                int: Result of the service
        """
        print("[INFO] Loading Image...")
        frame = self.load_img()

        # Resize image to be maximum 400px wide
        frame = imutils.resize(frame, width = 400)

        # Blob conversion (detecion purposes)
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

        # Run feed-forward
        print("[INFO] Detection...")
        self.net.setInput(blob)
        detections = self.net.forward()

        # loop over the detections
        for i in np.arange(0, detections.shape[2]):
            # Get detection probability
            confidence = detections[0, 0, i, 2]

            # Get ID of the detection object
            idx = int(detections[0, 0, i, 1])

            # Filter out non-human detection with low confidence
            if confidence > self.confidence and idx == self.target:
                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Rectangle keypoints
                top_left = (startX, startY)
                bottom_right = (endX, endY)

                # draw bounding box
                label = "{}: {:.2f}%".format(self.targets[idx], confidence * 100)
                roi = cv2.rectangle(frame, top_left, bottom_right, self.colours[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colours[idx], 2)

                # Get centre point of the rectangle and draw it
                centre_point = self.getCentre(top_left, bottom_right)
                cv2.circle(frame, centre_point, 4, (0,0,255), -1)

                # Create a custom details
                # message for every good
                # detection
                detection = Detection()
                detection.ID = self.number_of_detections
                detection.width = bottom_right[0] - top_left[0]
                detection.height = bottom_right[1] - top_left[1]
                detection.top_left_x = top_left[0]
                detection.top_left_y = top_left[1]
                detection.centre_x = centre_point[0]
                detection.centre_y = centre_point[1]

                # Aggregate the detection to the others
                self.detections.array.append(detection)

                # Human detections counter
                self.number_of_detections += 1

        # Save frame
        self.store(frame)

        # Add number_of_detections item to the detections message
        self.detections.number_of_detections = self.number_of_detections

        # Request depth mapping on the detections
        rospy.loginfo("Requesting depth mapping for the detections...")
        self.requestMapping(self.detections, req.depth)

        return RequestDetectionResponse("success")

    def requestMapping(self, detections, depth_image):
        """
            ROS service that requests the
            depth distance of the detections
            from the RGBD-optical frame.

            Arguments:
                detections: RGB detections rgb position
        """
        # Wait for service to come alive
        rospy.wait_for_service('rgb_to_depth_mapping')

        try:
            # Build request
            request = rospy.ServiceProxy('rgb_to_depth_mapping', RequestDepth)

            # Get response from service
            response = request(detections, depth_image)

            # Publish detections
            self.detection_pub.publish(self.detections)
            self.depth_pub.publish(response.distances)

            # Clean
            self.detections = Detections()
            self.number_of_detections = 0

        except rospy.ServiceException as e:
            rospy.loginfo("Depth service call failed: %s", e)

    def getDetectionObject(self, confidence, rgb_x, rgb_y):
        """
            Detection object for
            detection custom message
            population.

            Arguments:
                param1: detection confidence
                param2: x coordinate of the centre box
                param3: y coordinate of the centre box

            Returns:
                object: detection object
        """
        return {'confidence': confidence, 'rgb_x': rgb_x, 'rgb_y': rgb_y}

    # Load image to be processed
    def load_img(self):
        """
            Load image to be processed.

            Returns:
                image: MAT image
        """
        return cv2.imread(self.path + "/data/converted/image.png")

    def store(self, frame):
        """
            Stores image with
            detections' bounding
            boxes.

            Arguments:
                param1: MAT image with detection boxes
        """
        cv2.imwrite(self.path + "/data/detections/detections.png", frame)

    def getCentre(self, tl, br):
        """
            Finds centre point
            of the bounding box.

            Arguments:
                int: Top left corner of the rectangle
                int: Bottom right corner of the rectangle

            Returns:
                tuple of ints: X and Y coordinate of centre point
        """
        # Compute distances
        width  = br[0] - tl[0]
        height = br[1] - tl[1]

        # Return centre
        return (tl[0] + int(width * 0.5), tl[1] + int(height * 0.5))

def main(args):

    # Initialise node
    rospy.init_node('person_detection', anonymous=True)

    try:
        # Initialise
        hd = PersonDetection()

        # Detection service
        service = rospy.Service('detection', RequestDetection, hd.detection)

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
