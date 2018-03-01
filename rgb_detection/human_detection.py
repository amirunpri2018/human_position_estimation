#!/usr/bin/env python

"""
    Human Detection Service.

    ROS service that takes as input
    an image in a MAT format, and
    returns a frame with detections.
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

# Path module
from pathlib import Path

# Import ROS Image data type
from sensor_msgs.msg import Image

# Custom detection messages
from human_aware_robot_navigation.msg import *
from human_aware_robot_navigation.srv import *

class PersonDetection:

    def __init__(self):
        """
            Constructor.
        """
        # Detection target (person)
        self.target = 15

        # Confidence (for detection)
        self.confidence = 0.7

        # Number of human detections
        self.number_of_detections = 0

        # Detection messages
        self.msg_detection = Detection()
        self.msg_detections = Detections()

        # Publishing rate
        self.rate = rospy.Rate(10)

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
        self.detection_pub = rospy.Publisher('person_detection', Detections)

        print("[INFO] Successful Initialisation")

    def detection(self, req):
        """
            Returns the frame with
            detections bounding boxes.

            Params:
                req: Service request

            Ouput:
                int: Result of the service
        """
        # Load image to be processed
        print("[INFO] Loading Image...")
        frame = self.load_img()

        # Resize image to be maximum 400px wide
        frame = imutils.resize(frame, width = 400)

        # grab the frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

        # pass the blob through the network and obtain the detections and
        # predictions
        print("[INFO] Detection...")
        self.net.setInput(blob)
        detections = self.net.forward()

        # loop over the detections
        for i in np.arange(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with
            # the prediction
            confidence = detections[0, 0, i, 2]

            # extract the index of the class label from the
            idx = int(detections[0, 0, i, 1])

            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            if confidence > self.confidence and idx == self.target:
                # Human detections counter
                self.number_of_detections += 1

                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Rectangle keypoints
                top_left = (startX, startY)
                bottom_right = (endX, endY)

                # draw the prediction on the frame
                label = "{}: {:.2f}%".format(self.targets[idx], confidence * 100)
                cv2.rectangle(frame, top_left, bottom_right, self.colours[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colours[idx], 2)

                # Get centre point in
                # the rectangle
                print("Top left: ", top_left)
                print("Bottom right: ", bottom_right)
                centre_point = self.getCentre(top_left, bottom_right)
                print("Centre point: ", centre_point)

                # Populate Details message
                details = Details()
                details.confidence = confidence
                details.height = 480
                details.width = 640
                details.rgb_x = centre_point[0]
                details.rgb_y = centre_point[1]

                # Populate Detection message
                self.msg_detection.details.append(details)

                # Save frame
                self.store(frame)

                # Draw circle
                cv2.circle(frame, centre_point, 63, (0,0,255), -1)

                # Show image
                # cv2.imshow('image', frame)
                # cv2.waitKey(5)

        # Publish message
        self.publishDetections()

        # Return service response
        return RequestDetectionResponse("success")

    def publishDetections(self):
        """
            Populates detections array,
            sends it and clears it afterwards
            along with the detection array.
        """
        # Populate detections (array of detection)
        self.msg_detections.detections.append(self.msg_detection)
        self.msg_detections.number_of_detections = self.number_of_detections

        # Publish detections
        self.detection_pub.publish(self.msg_detections)

        # Clean detections and detection arrays
        self.msg_detection = Detection()
        self.msg_detections = Detections()

        # Clear number of detections
        self.number_of_detections = 0

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
        return cv2.imread(self.path + "/data/converted/image.jpg")

    def store(self, frame):
        """
            Stores image with
            detections' bounding
            boxes.

            Arguments:
                param1: MAT image with detection boxes
        """
        cv2.imwrite(self.path + "/data/detections/detections.jpg", frame)

    def getCentre(self, tl, br):
        """
            Finds centre point
            of the bounding box.

            Arguments:
                param1: Top left corner of the rectangle
                param2: Bottom right corner of the rectangle

            Returns:
                point: Centre point of the rectangle
        """
        # Compute distances
        width  =
        height =

        # Radian angle
        theta_rad = math.radians(45)

        # Degree conversion of cosine
        cos_deg = math.degrees(math.cos(theta_rad))

        # centre point
        centreOffset = int(pk_distance/2 * cos_deg)

        # Return tuple of the centre
        return (p[0] + centreOffset, p[1] + centreOffset)

    def getDistance(self, point1, point2):
        """
            Computes euclidean distance
            between two points.

            Arguments:
                param1: Point1
                param2: Point2
1
            Returns:
                distance: euclidean distance between the points
        """
        x_squared_diff = math.pow((point1[0] - point2[0]), 2)
        y_squared_diff = math.pow((point1[1] - point2[1]), 2)
        return math.sqrt(x_squared_diff + y_squared_diff)

def main(args):

    # Initialise node
    rospy.init_node('detection_server', anonymous=True)

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
