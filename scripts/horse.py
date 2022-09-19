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

        hsv = cv.cvColor(image, cv.COLORO_BGR2HSV) # change BGR to HSV
        lower_yellow = np.array([10, 10, 10]) # lower limit
        upper_yellow = np.array([255, 255, 255]) # upper limit
        mask = cv.inRange(hsv, lower_yellow, upper_yellow) # binarization
        masked = cv.bitwise_and(image, image, mask = mask) # filtering(leave mask's part of 1)

        h, w = image.shape[:2]
        RESIZE = (w//3, h//3)
        search_top = (h//4)*3
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:h] = 0

        M = cv.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) #

        cv.imshow('MASK', image) #
        cv.waitkey(3) # wait 3ms

rospy.init_node('horse')
horse = Horse()
rospy.spin()