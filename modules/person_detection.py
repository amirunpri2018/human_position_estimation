"""
    Detection.

    The class is able to detect the
	presence of humans in the image
	feed of the robot.
"""

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
import numpy as np
import argparse
import imutils
import time
import cv2

class PersonDetection:

    def __init__(self):
		""" Class constructor """

		# Define detection's target/s
	    self.target = ["person"]

        # Bounding boxes colours
	    self.colours = np.random.uniform(0, 255, size=(len(self.target), 3))

		# Load NN's serialised model
		print("[INFO] loading model...")
		self.net = cv2.dnn.readNetFromCaffe("parameters/MobileNetSSD_deploy.prototxt.txt",
											"parameters/MobileNetSSD_deploy.caffemodel")

	def detect(frame):
		"""
			Returns the frame with recognitions.
		"""

		# Resize image to be maximum 400px wide
		frame = imutils.resize(frame, width = 400)

		# grab the frame dimensions and convert it to a blob
		(h, w) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

		# pass the blob through the network and obtain the detections and
		# predictions
		net.setInput(blob)
		detections = net.forward()

		# loop over the detections
		for i in np.arange(0, detections.shape[2]):
			# extract the confidence (i.e., probability) associated with
			# the prediction
			confidence = detections[0, 0, i, 2]

			# filter out weak detections by ensuring the `confidence` is
			# greater than the minimum confidence
			if confidence > args["confidence"]:
				# extract the index of the class label from the
				# `detections`, then compute the (x, y)-coordinates of
				# the bounding box for the object
				idx = int(detections[0, 0, i, 1])
				box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
				(startX, startY, endX, endY) = box.astype("int")

				# draw the prediction on the frame
				label = "{}: {:.2f}%".format(self.target[idx],
					confidence * 100)
				cv2.rectangle(frame, (startX, startY), (endX, endY),
					self.colours[idx], 2)
				y = startY - 15 if startY - 15 > 15 else startY + 15
				cv2.putText(frame, label, (startX, y),
					cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.colours[idx], 2)

		# show the output frame
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF

		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			break

		# update the FPS counter
		fps.update()

	# stop the timer and display FPS information
	fps.stop()
	print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
	print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

	# do a bit of cleanup
	cv2.destroyAllWindows()
	vs.stop()
