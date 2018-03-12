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
from message_filters import TimeSynchronizer, Subscriber

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
        self.msg_detection = Detection()
        self.msg_detections = Detections()

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
        self.detection_pub = rospy.Publisher('detections', Detections)

        # Publisher (custom detection message)
        self.depth_pub = rospy.Publisher('distances', Distances)

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

                # Get centre point in the rectangle
                centre_point = self.getCentre(top_left, bottom_right)
                # print("Top left: ", top_left)
                # print("Bottom right: ", bottom_right)
                print("Centre point: ", centre_point)

                # Populate Details message
                detail = Details()
                detail.ID = self.number_of_detections
                detail.rgb_x = centre_point[0]
                detail.rgb_y = centre_point[1]
                detail.confidence = confidence

                # Populate Detection message
                self.msg_detection.details.append(detail)

                # Draw circle
                cv2.circle(frame, centre_point, 4, (0,0,255), -1)

                # Human detections counter
                self.number_of_detections += 1

        # Save frame
        self.store(frame)

        # Publish message
        self.publishDetections()

        # Return service response
        return RequestDetectionResponse("success")

    def publishDetections(self):
        """
            Populates detections array, requests
            depth measurements for each detection
            and publishes the topics. Clears data
            afterwards.
        """
        # Populate detections message (array of detection)
        self.msg_detections.detections.append(self.msg_detection)
        self.msg_detections.number_of_detections = self.number_of_detections

        # Request depth data
        rospy.loginfo("Requesting depth data for the detections...")
        depth_res = self.requestDepths(self.msg_detections)

        # Publish detections
        self.detection_pub.publish(self.msg_detections)
        self.depth_pub.publish(depth_res)

        # Clean detections and detection arrays
        self.msg_detection = Detection()
        self.msg_detections = Detections()

        # Clear number of detections
        self.number_of_detections = 0

    def requestDepths(self, detections):
        """
            ROS service that requests the
            depth distance of the detections
            from the RGBD-optical frame.

            Arguments:
                detections: RGB detections rgb position
        """
        # Wait for service to come alive
        rospy.wait_for_service('distance')

        try:
            # Build request
            request = rospy.ServiceProxy('distance', RequestDepth)

            # Get response from service
            response = request(detections)

            # Response
            return response.res

        except Exception as e:
            rospy.loginfo("Error during human detection request: %s", e)

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
                param1: Top left corner of the rectangle
                param2: Bottom right corner of the rectangle

            Returns:
                point: Centre point of the rectangle
        """
        # Compute distances
        width  = br[0] - tl[0]
        height = br[1] - tl[1]

        # Return centre
        return (tl[0] + int(width * 0.5), tl[1] + int(height * 0.5))

    def callback(self, depth_raw_image):
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

    def setDepthImage(self, depth_image):
        """
            Stores converted
            depth image.
        """
        self.depth_image = depth_image

    # def getDepthImage(self):
    #     """
    #         Returns converted
    #         depth image.
    #     """
    #     return self.depth_image

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
