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
import imutils
import numpy as np

# Path module
from pathlib import Path

def store(frame):
    """
        Stores image with
        detections.

        Arguments:
            param1: MAT image with detection boxes
    """
    cv2.imwrite(str(Path(os.getcwd()).parents[0]) + "/data/detections/detections.jpg", frame)

# Load image to be processed
def load_img():
    """
        Load image to be processed.

        Returns:
            image: MAT image
    """
    return cv2.imread(str(Path(os.getcwd()).parents[0]) + "/converted/image.jpg")

def person_detection():
    """
        Returns the frame with
        detections bounding boxes.

        Params:
            MAT: Image with MAT format

        Ouput:
            MAT: Image with detections boxes
    """
    # Define detection's target/s
    targets = ["person"]

    # Bounding boxes colours
    colours = np.random.uniform(0, 255, size=(len(targets), 3))

    # Load NN's serialised model
    print("[INFO] Loading Neural Network...")
    net = cv2.dnn.readNetFromCaffe(str(Path(os.getcwd()).parents[0]) + "/data/nn_params/MobileNetSSD_deploy.prototxt.txt",
                                   str(Path(os.getcwd()).parents[0]) + "/data/nn_params/MobileNetSSD_deploy.caffemodel")

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
    net.setInput(blob)
    detections = net.forward()

    # loop over the detections
    for i in np.arange(0, detections.shape[2]):
        # extract the confidence (i.e., probability) associated with
        # the prediction
        confidence = detections[0, 0, i, 2]

        # filter out weak detections by ensuring the `confidence` is
        # greater than the minimum confidence
        if confidence > 0.2:
            # extract the index of the class label from the
            # `detections`, then compute the (x, y)-coordinates of
            # the bounding box for the object
            idx = int(detections[0, 0, i, 1])
            box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
            (startX, startY, endX, endY) = box.astype("int")

            # draw the prediction on the frame
            label = "{}: {:.2f}%".format(targets[idx], confidence * 100)
            cv2.rectangle(frame, (startX, startY), (endX, endY), colours[idx], 2)
            y = startY - 15 if startY - 15 > 15 else startY + 15
            cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, colours[idx], 2)

    store(frame)

def main(args):

    # Initialise node
    rospy.init_node('detection', anonymous=True)

    try:
        # Perform detection
        person_detection()

        # Spin it baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)

# Execute main
if __name__ == '__main__':
    main(sys.argv)
