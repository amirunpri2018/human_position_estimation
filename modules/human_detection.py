#!/usr/bin/env python

"""
    Human Detection Service.

    ROS service that takes as input
    an image in a MAT format, and
    returns a frame with detections.
"""

# import the necessary packages
import cv2
import imutils
import numpy as np

def handle_person_detection(self, frame):
    """
        Returns the frame with
        detections bounding boxes.

        Params:
            MAT: Image with MAT format

        Ouput:
            MAT: Image with detections boxes
    """

    print("Request: ", frame)

    # Define detection's target/s
    targets = ["person"]

    # Bounding boxes colours
    colours = np.random.uniform(0, 255, size=(len(targets), 3))

    # Load NN's serialised model
    net = cv2.dnn.readNetFromCaffe("/home/itaouil/tiago_ws/src/human_aware_robot_navigation/src/HARN/modules/MobileNetSSD_deploy.prototxt.txt",
                                   "/home/itaouil/tiago_ws/src/human_aware_robot_navigation/src/HARN/modules/MobileNetSSD_deploy.caffemodel")

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
        if confidence > 0.8:
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

    return RunDetectionResponse(frame)

def main(args):

    try:
        # Setting up service
        print("[INFO] Detection service set up...")

        # Start node
        rospy.init_node('person_detection')

        # Create service
        s = rospy.Service('person_detection', RunDetection, handle_person_detection)

        # Log acknowledgement
        print("[INFO] Ready to detect humans...")

        # Spin it, baby !
        rospy.spin()

    except KeyboardInterrupt as e:
        print('Error during main execution' + e)
