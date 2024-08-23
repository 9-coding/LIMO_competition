#! /usr/bin/env python
# -*- coding: utf-8 -*-
# limo_application/scripts/lane_detection/lane_detect.py
# WeGo LIMO Pro를 이용한 차선 인식 코드

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, String  # String 메시지 타입을 임포트
from dynamic_reconfigure.server import Server
from limo_application.cfg import image_processingConfig

import cv2
import numpy as np

class LaneDetection:
    def __init__(self):
        rospy.init_node("lane_detect")
        srv = Server(image_processingConfig, self.reconfigure_callback)
        self.cvbridge = CvBridge()
        rospy.Subscriber(rospy.get_param("~image_topic_name", "/camera/rgb/image_raw/compressed"), CompressedImage, self.image_topic_callback)
        self.mode = "default"
        rospy.Subscriber("/mode", String, self.mode_CB)
        self.distance_pub = rospy.Publisher("/limo/lane_x", Int32, queue_size=5)
        self.viz = rospy.get_param("~visualization", True)
        self.active = True
        self.LANE_WIDTH_THRESHOLD = rospy.get_param("~lane_width_threshold", 100)  # 가로 폭 임계값 설정

    def mode_CB(self, msg):
        self.mode = msg.data
        if self.mode == "lane":
            self.active = True
        elif self.mode == "obstacle":
            self.active = False

    def imageCrop(self, _img=np.ndarray(shape=(480, 640))):
        if self.mode == "lane":
            return _img[420:480, 0:320]
        else:
            return _img[420:480, 0:320]

    def colorDetect(self, _img=np.ndarray(shape=(480, 640))):
        hls = cv2.cvtColor(_img, cv2.COLOR_BGR2HLS)
        mask_yellow = cv2.inRange(hls, self.YELLOW_LANE_LOW_TH, self.YELLOW_LANE_HIGH_TH)
        mask_white = cv2.inRange(hls, self.WHITE_LANE_LOW_TH, self.WHITE_LANE_HIGH_TH)
        edges = cv2.Canny(hls, 50, 150)
        mask_lane = cv2.bitwise_or(mask_yellow, mask_white)
        combined_mask_lane_and_edge = cv2.bitwise_or(mask_lane, edges)
        return combined_mask_lane_and_edge

    def calcLaneDistance(self, _img=np.ndarray(shape=(480, 640))):
        try:
            M = cv2.moments(_img)
            self.x = int(M['m10']/M['m00'])
            self.y = int(M['m01']/M['m00'])

            # 차선의 가장 왼쪽과 오른쪽 경계 계산
            left_edge = np.min(np.where(_img[self.y, :] > 0))
            right_edge = np.max(np.where(_img[self.y, :] > 0))
            lane_width = right_edge - left_edge

            # 가로 폭이 임계값을 초과하면 차선 인식하지 않음
            if lane_width > self.LANE_WIDTH_THRESHOLD:
                self.x = -1
                self.y = -1
        except:
            self.x = -1
            self.y = -1
        return self.x

    def visResult(self):
        if self.x != -1 and self.y != -1:
            cv2.circle(self.cropped_image, (self.x, self.y), 10, 255, -1)
        cv2.imshow("lane_original", self.frame)
        cv2.imshow("lane_cropped", self.cropped_image)
        cv2.imshow("lane_thresholded_edge", self.thresholded_image)
        cv2.waitKey(1)

    def reconfigure_callback(self, config, level):
        self.YELLOW_LANE_LOW_TH = np.array([config.yellow_h_low, config.yellow_l_low, config.yellow_s_low])
        self.YELLOW_LANE_HIGH_TH = np.array([config.yellow_h_high, config.yellow_l_high, config.yellow_s_high])
        self.WHITE_LANE_LOW_TH = np.array([config.white_h_low, config.white_l_low, config.white_s_low])
        self.WHITE_LANE_HIGH_TH = np.array([config.white_h_high, config.white_l_high, config.white_s_high])
        return config

    def image_topic_callback(self, img):
        if not self.active:
            return

        self.frame = self.cvbridge.compressed_imgmsg_to_cv2(img, "bgr8")
        self.cropped_image = self.imageCrop(self.frame)
        self.thresholded_image = self.colorDetect(self.cropped_image)
        self.left_distance = self.calcLaneDistance(self.thresholded_image)
        self.distance_pub.publish(self.left_distance)

        if self.viz:
            self.visResult()
         
def run():
    new_class = LaneDetection()
    rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except KeyboardInterrupt:
        print("program down")
