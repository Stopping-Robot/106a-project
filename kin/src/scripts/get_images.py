#!/usr/bin/env python
from __future__ import print_function

import sys
import os
import rospy
import cv2
import numpy as np
from image_geometry import PinholeCameraModel
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
import imutils



class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()

    #Adi: Use this topic for Sawyer right hand camera feed
    #self.image_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw",Image,self.callback)

    #Adi: Use this topic for realsense color
    #self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

    #Adi: Use this topic for realsense depth
    #self.image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image,self.callback)

    #Adi: Use this topic for Baxter left_hand_camera
    self.image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image,self.callback)

    #Adi: Define the camera model
    self.cam_info = rospy.wait_for_message("/cameras/left_hand_camera/camera_info", CameraInfo, timeout=5)
    self.cam_model = PinholeCameraModel()
    self.cam_model.fromCameraInfo(self.cam_info)

    ##Adi: Load YOLO parameters and pre-trained model 
    #model_path = "yolo-object-detection/yolo-coco"
    #
    ## load the COCO class labels our YOLO model was trained on
    #self.labelsPath = os.path.sep.join([model_path, "coco.names"])
    #self.LABELS = open(self.labelsPath).read().strip().split("\n")

    ## derive the paths to the YOLO weights and model configuration
    #self.weightsPath = os.path.sep.join([model_path, "yolov3.weights"])
    #self.configPath = os.path.sep.join([model_path, "yolov3.cfg"])

    ## load our YOLO object detector trained on COCO dataset (80 classes)
    #self.net = cv2.dnn.readNetFromDarknet(self.configPath, self.weightsPath)

    # initialize the list of class labels MobileNet SSD was trained to
    # detect, then generate a set of bounding box colors for each class
    self.prototxt = "real-time-object-detection/MobileNetSSD_deploy.prototxt.txt"
    self.model = "real-time-object-detection/MobileNetSSD_deploy.caffemodel"
    self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
        "sofa", "train", "tvmonitor"]
    self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))


    # load our serialized model from disk
    self.net = cv2.dnn.readNetFromCaffe(self.prototxt, self.model)

  def resize(self, image, width=None, height=None, inter=cv2.INTER_AREA):
      # initialize the dimensions of the image to be resized and
      # grab the image size
      dim = None
      (h, w) = image.shape[:2]
  
      # if both the width and height are None, then return the
      # original image
      if width is None and height is None:
          return image
  
      # check to see if the width is None
      if width is None:
          # calculate the ratio of the height and construct the
          # dimensions
          r = height / float(h)
          dim = (int(w * r), height)
  
      # otherwise, the height is None
      else:
          # calculate the ratio of the width and construct the
          # dimensions
          r = width / float(w)
          dim = (width, int(h * r))
  
      # resize the image
      resized = cv2.resize(image, dim, interpolation=inter)
  
      # return the resized image
      return resized
    
  def grab_contours(self, cnts):
      # if the length the contours tuple returned by cv2.findContours
      # is '2' then we are using either OpenCV v2.4, v4-beta, or
      # v4-official
      if len(cnts) == 2:
          cnts = cnts[0]
  
      # if the length of the contours tuple is '3' then we are using
      # either OpenCV v3, v4-pre, or v4-alpha
      elif len(cnts) == 3:
          cnts = cnts[1]
  
      # otherwise OpenCV has changed their cv2.findContours return
      # signature yet again and I have no idea WTH is going on
      else:
          raise Exception(("Contours tuple must have length 2 or 3, "
              "otherwise OpenCV changed their cv2.findContours return "
              "signature yet again. Refer to OpenCV's documentation "
              "in that case"))
  
      # return the actual contours array
      return cnts

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      #cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
      #print(cv_image)
      #cv2.imshow("image", cv_image)
    except CvBridgeError as e:
      print(e)

#    (h, w) = cv_image.shape[:2]
#    blob = cv2.dnn.blobFromImage(cv2.resize(cv_image, (300, 300)), 0.007843, (300, 300), 127.5) 
#
#    self.net.setInput(blob)
#    detections = self.net.forward()
#
#    # loop over the detections
#    for i in np.arange(0, detections.shape[2]):
#	# extract the confidence (i.e., probability) associated with
#	# the prediction
#	confidence = detections[0, 0, i, 2]
# 
#	# filter out weak detections by ensuring the `confidence` is
#	# greater than the minimum confidence
#	if confidence > 0.05:
#        #if True:
#		# extract the index of the class label from the
#		# `detections`, then compute the (x, y)-coordinates of
#		# the bounding box for the object
#		idx = int(detections[0, 0, i, 1])
#		box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
#		(startX, startY, endX, endY) = box.astype("int")
# 
#		# draw the prediction on the frame
#		label = "{}: {:.2f}%".format(self.CLASSES[idx], confidence * 100)
#		cv2.rectangle(cv_image, (startX, startY), (endX, endY), self.COLORS[idx], 2)
#		y = startY - 15 if startY - 15 > 15 else startY + 15
#		cv2.putText(cv_image, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS[idx], 2)

    #print(cv_image.shape)
    #cam_coord = self.cam_model.projectPixelTo3dRay((640,400))
    #print(cam_coord)
    #print(cv_image[320][240])
    #(rows,cols, channel) = cv_image.shape
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (640,400), 10, 255)

    greenLower = (0, 0, 250)
    greenUpper = (0, 0, 255)
    #greenLower = (100, 130, 200)
    #greenUpper = (110, 190, 280)
    #pts = deque(maxlen=args["buffer"])
    pts = deque(maxlen=64)

    # resize the frame, blur it, and convert it to the HSV
    # color space
    cv_image = self.resize(cv_image, width=600)
    blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)
    cv2.imshow("blurred", blurred)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv", hsv)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, greenLower, greenUpper)
    cv2.imshow("mask", mask)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = self.grab_contours(cnts)
    center = None
 
    # only proceed if at least one contour was found
    if len(cnts) > 0:
	# find the largest contour in the mask, then use
	# it to compute the minimum enclosing circle and
	# centroid
	c = max(cnts, key=cv2.contourArea)
	((x, y), radius) = cv2.minEnclosingCircle(c)
	M = cv2.moments(c)
	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
	# only proceed if the radius meets a minimum size
	if radius > 10:
		# draw the circle and centroid on the frame,
		# then update the list of tracked points
		cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
		cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
 
    # update the points queue
    pts.appendleft(center)
    # loop over the set of tracked points
    for i in range(1, len(pts)):
	# if either of the tracked points are None, ignore
	# them
	if pts[i - 1] is None or pts[i] is None:
		continue
 
	# otherwise, compute the thickness of the line and
	# draw the connecting lines
	thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
	cv2.line(cv_image, pts[i - 1], pts[i], (0, 0, 255), thickness)


    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)


    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)

def main(args):
  rospy.init_node('camera', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
