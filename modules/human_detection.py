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
import rospy
import imutils
import numpy as np

# Path module
from pathlib import Path

# Import ROS Image data type
from sensor_msgs.msg import Image

class PersonDetection:

    def __init__(self):

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

        # Subscribe to TIAGo's image_raw topic
        self.image_sub = rospy.Subscriber('xtion/rgb/image_raw', Image, self.person_detection)

        print("[INFO] Successful Initialisation")

    def person_detection(self, data):
        """
            Returns the frame with
            detections bounding boxes.

            Params:
                MAT: Image with MAT format

            Ouput:
                MAT: Image with detections boxes
        """
        # Load image to be processed
        print("[INFO] Loading Image...")
        frame = load_img()

        # Resize image to be maximum 400px wide
        frame = imutils.resize(frame, width = 400)

        # grab the frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

        # pass the blob through the network and obtain the detections and
        # predictions
        print("[INFO] Running detection...")
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
            if confidence > 0.2 and idx == 15:
                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # draw the prediction on the frame
                label = "{}: {:.2f}%".format(self.targets[idx], confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY), self.colours[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colours[idx], 2)

                # Save frame
                store(frame)

                # Show image
                cv2.imshow('image', frame)
                cv2.waitKey(5)

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
            detections.

            Arguments:
                param1: MAT image with detection boxes
        """
        cv2.imwrite(self.path + "/data/detections/detections.jpg", frame)

def main(args):

    # Initialise node
    rospy.init_node('detection', anonymous=True)

    try:
        # Initialise
        hd = PersonDetection()

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
