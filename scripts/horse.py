#!/usr/bin/env python
import rospy, cv_bridge
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np

class Horse:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv.namedWindow('BGR Image', 1) # create window named BGR Image
        cv.namedWindow('MASK', 1) # create window named MASK
        cv.namedWindow('MASKED', 1) # create window named MASKED
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback) # Image Subscriber

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8') # Passing Image to OpenCV
        image = cv.resize(image, (image.shape[1]//2, image.shape[0]//2)) # resize Image
        cv.imshow('MASK', image) #
        cv.waitkey(3) # wait 3ms

rospy.init_node('horse')
horse = Horse()
rospy.spin()