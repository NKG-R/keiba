#!/usr/bin/env python
import rospy, cv_bridge
from sensor_msgs.msg import Image
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Twist
import random

class Horse:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv.namedWindow('BGR Image', 1) # create window named BGR Image
        cv.namedWindow('MASK', 1) # create window named MASK
        cv.namedWindow('MASKED', 1) # create window named MASKED
        self.image_sub = rospy.Subscriber('/beego/my_robo/camera1/image_raw', Image, self.image_callback) # Image Subscriber
        self.cmd_vel_pub = rospy.Publisher('/beego/diff_drive_controller/cmd_vel', Twist, queue_size=1) # cmd_vel Publisher

        self.max_speed = random.uniform(0.8, 1.5)
        self.min_speed = random.uniform(0.0, 0.4)
        self.speed = random.uniform(0.6, 1.5)
        self.kp = random.uniform(500, 1000)
        self.cnt = 0

        self.twist = Twist()

        print(self.max_speed)
        print(self.min_speed)
        print(self.speed)
        print(self.kp)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8') # Passing Image to OpenCV

        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV) # change BGR to HSV
        lower = np.array([0, 0, 0]) # lower limit
        upper = np.array([255, 100, 100]) # upper limit
        mask = cv.inRange(hsv, lower, upper) # binarization
        masked = cv.bitwise_and(image, image, mask = mask) # filtering(leave mask's part of 1)

        h, w = image.shape[:2]
        RESIZE = (w//3, h//3)
        search_top = (h//6)*5
        search_bot = search_top + 15
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv.moments(mask) # centroid of part 1 in mask
        if M['m00'] > 0: # if exist centroid
            cx = int(M['m10']/M['m00']) # centroid x
            cy = int(M['m01']/M['m00']) # centroid y
            cv.circle(image, (cx, cy), 20, (0, 0, 255), -1) # draw centroid point
        
            # P control
            err = cx - w//2 # difference between centroid x and center of image x 
            
            if self.cnt > 100:
                self.speed = random.uniform(0.0, 1.5)
                if self.speed > self.max_speed:
                    self.speed = self.max_speed
                elif self.speed < self.min_speed:
                    self.speed = self.min_speed
                print(self.speed)
                self.cnt = 0

            self.twist.linear.x = self.speed
            self.twist.angular.z = -float(err)/self.kp
            self.cmd_vel_pub.publish(self.twist)
            self.cnt = self.cnt + 1
            print(self.cnt)

        # resize
        display_mask = cv.resize(mask, RESIZE)
        display_masked = cv.resize(masked, RESIZE)
        display_image = cv.resize(image, RESIZE)

        # display
        cv.imshow('BGR Image', display_image) # display BGR Image
        cv.imshow('MASK', display_mask) # display MASK
        cv.imshow('MASKED', display_masked) # display MASKED
        cv.waitKey(3) # wait 3ms

rospy.init_node('horse')
horse = Horse()
rospy.spin()