#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
import CMT
import numpy as np
import util

from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
import geometry_msgs.msg as geometry_msgs
from cv_bridge import CvBridge, CvBridgeError

class Extract_center:
    def __init__(self):
        # self.image_pub = rospy.Publisher("image_topic_2",Image)
        self.CMT = CMT.CMT()
        self.bridge = CvBridge()

        self.first = True
        self.rect = None
        self.init_img = None

        self.center = None
        self.prev_center = None

        self.h = None
        self.w = None

        self.linear_speed_x = 0.1
        self.k_yaw = 0.0005
        self.k_alt = 0.0005

        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image,self.callback)

        self.des_vel_pub = rospy.Publisher("/rexrov/cmd_vel", numpy_msg(geometry_msgs.Twist), queue_size=1)

    def tem_match(self, orig, src, templ):
        img = src
        img2 = img.copy()
        template = templ
        w, h = template.shape[::-1]
        tl = bl = None
        point = None
        methods = ['cv2.TM_CCOEFF_NORMED']
        for meth in methods:
            img = img2.copy()
            resize_i = img2.copy()
            method = eval(meth)
            orig_res = None
            for i in range(1):
                resize_i = cv2.resize(img, None,fx=1/2**(0.5*i), fy=1/2**(0.5*i), interpolation = cv2.INTER_AREA)
                print(resize_i.shape)
                # Apply template Matching
                res = cv2.matchTemplate(resize_i, template, method)
                if i == 0:
                    orig_res = res
                # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                # tl = max_loc
                # br = (tl[0] + w, tl[1] + h)
                # cv2.rectangle(orig,tl, br, 255, 2)
                threshold = 0.70
                loc = np.where( res >= threshold)
                for pt in zip(*loc[::-1]):
                    cv2.rectangle(orig, (pt[0]*int(2**(0.5*i)),pt[1]*int(2**(0.5*i))), ((pt[0] + w), (pt[1] + h)), (0,0,255), 1)
                    point = pt

        try:
            tl = (point[0]*int(2**(0.5*i)),point[1]*int(2**(0.5*i)))
            br = ((point[0] + w), (point[1] + h))
            # self.tl = tl
            # self.br = br
        except Exception() as e:
            print(e)

        center = ((br[0]-tl[0])/2 , (br[1]-tl[1])/2)
        cv2.circle(orig,(tl[0] + center[0], tl[1] + center[1]), 1, (0,0,255), 2)
        cv2.imshow('Matching Result', orig_res)
        cv2.imshow('Detected Point', orig)
        cv2.waitKey(100)
        return tl, br

    def contour_detection(self, orig, gray):
        cv2.imshow('Gray Point', gray)
        ret, thresh = cv2.threshold(gray, 160, 255, 0)
        cv2.imshow('Thresh Point', thresh)
        im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = contours[4]
        cv2.drawContours(orig, [cnt[0]], 0, (0,255,0), 3)
        # self.tl = cnt[0]
        cv2.imshow('Contour Point', orig)
        cv2.waitKey(100)

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
        if self.first:
            self.init_img = cv_image
            self.first = False
            self.h = cv_image.shape[0]
            self.w = cv_image.shape[1]
            print("h: " + str(self.h) + " "+ "w: " + str(self.w) )
            tl, br = self.extract_init_img(self.init_img)
            print(tl)
            print(br)
            # tl = (437, 143)
            # br = (500, 206)
            # cv2.rectangle(self.init_img, tl, br, 255, 2)
            # cv2.imshow("try", self.init_img)
            # cv2.waitKey(100)
            self.CMT.initialise(self.init_img, tl, br)
        else:
            self.track(cv_image)
            self.target_follower()

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def target_follower(self):
        msg = geometry_msgs.Twist()
        d_alt = self.k_alt*(self.h/2 - self.center[1])
        d_yaw = self.k_yaw*(self.w/2 - self.center[0])

        msg.linear.x = self.linear_speed_x
        msg.linear.y = 0
        msg.linear.z = d_alt

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = d_yaw

        print(msg)

        self.des_vel_pub.publish(msg)

    def extract_init_img(self, cv_frame):
        buoy = cv2.imread('red1.png')
        templ_gray = cv2.cvtColor(buoy, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.cvtColor(cv_frame, cv2.COLOR_BGR2GRAY)
        self.h , self.w = frame_gray.shape
        tl, br = self.tem_match(cv_frame, frame_gray, templ_gray)
        # self.contour_detection(self.init_img, frame_gray)
        return tl, br

    def track(self, cv_image):
        self.CMT.process_frame(cv_image)
        #
        # tl_x = int(self.CMT.tl[0])
        # tl_y = int(self.CMT.tl[1])
        #
        # br_x = int(self.CMT.br[0])
        # br_y = int(self.CMT.br[1])
        # cv2.rectangle(cv_image, (tl_x, tl_y),(br_x, br_y), (255, 0, 0), 4)
        #
        try:
            # tl, br = self.extract_init_img(cv_frame)
            tl = self.CMT.tl
            br = self.CMT.br
            self.prev_center = self.center
            self.center = center = ( tl[0] + (br[0]-tl[0])/2 , tl[1] + (br[1]-tl[1])/2)
            print("center " + str(self.center))
            tl_x = int(self.CMT.tl[0])
            tl_y = int(self.CMT.tl[1])

            br_x = int(self.CMT.br[0])
            br_y = int(self.CMT.br[1])
            cv2.rectangle(cv_image, (tl_x, tl_y),(br_x, br_y), (255, 0, 0), 4)
            cv2.circle(cv_image,(center[0], center[1]), 1, (255,0,0), 2)
            cv2.circle(cv_image,(self.w/2, self.h/2), 1, (0,255,0), 2)
        except Exception() as e:
            print(e)
            # print(tl)
            # print(br)
            self.center = self.prev_center

def main(args):
  ec = Extract_center()
  rospy.init_node('target_follower', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
