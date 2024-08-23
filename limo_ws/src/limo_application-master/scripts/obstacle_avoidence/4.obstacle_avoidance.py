#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # String 메시지 타입을 임포트
from math import *


class Limo_obstacle_avoidence:
    def __init__(self):
        rospy.init_node("laser_scan_node")
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=3)
        self.mode_pub = rospy.Publisher('/mode', String, queue_size=10)
        self.rate = rospy.Rate(30)

        self.cmd_vel_msg = Twist()
        self.msg = None
        self.lidar_flag = False
        self.dist_data = 0
        self.direction = None
        self.is_scan = False

        self.obstacle_ranges = []
        self.center_list_left = []
        self.center_list_right = []

        self.scan_dgree = 40
        self.min_dist = 0.2

        self.speed = 0
        self.angle = 0
        self.default_speed = 0.2
        self.default_angle = 0.0
        self.turning_speed = 0.08
        self.backward_speed = -0.15

        self.OBSTACLE_PERCEPTION_BOUNDARY = 20
        self.obstacle_exit = False
        self.ranges_length = None

        # 이전 상태를 저장할 변수 초기화
        self.previous_mode = None

    def laser_callback(self, msg):
        self.msg = msg

        if len(self.obstacle_ranges) > self.OBSTACLE_PERCEPTION_BOUNDARY:
            self.obstacle_exit = True
        else:
            self.obstacle_exit = False
        self.is_scan = True
        self.obstacle_ranges = []

    def LiDAR_scan(self):
        obstacle = []
        if not self.lidar_flag:
            self.degrees = [
                (self.msg.angle_min + (i * self.msg.angle_increment)) * 180 / pi
                for i, data in enumerate(self.msg.ranges)
            ]
            self.ranges_length = len(self.msg.ranges)
            self.lidar_flag = True

        for i, data in enumerate(self.msg.ranges):
            if 0 < data < 0.3 and -self.scan_dgree < self.degrees[i] < self.scan_dgree:
                obstacle.append(i)
                self.dist_data = data

        if obstacle:
            first = obstacle[0]
            first_dst = first
            last = obstacle[-1]
            last_dst = self.ranges_length - last
            self.obstacle_ranges = self.msg.ranges[first : last + 1]
        else:
            first, first_dst, last, last_dst = 0, 0, 0, 0

        return first, first_dst, last, last_dst

    def move_direction(self, last, first):
        if self.direction == "right":
            for i in range(first):
                self.center_list_left.append(i)
            Lcenter = self.center_list_left[floor(first / 2)]
            center_angle_left = -self.msg.angle_increment * Lcenter
            self.angle = center_angle_left
            self.speed = self.default_speed

        elif self.direction == "left":
            for i in range(len(self.msg.ranges)):
                self.center_list_right.append(last + i)
            Rcenter = self.center_list_right[
                floor(last + (self.ranges_length - last) / 2)
            ]
            center_angle_right = self.msg.angle_increment * Rcenter
            self.angle = center_angle_right / 2.5
            self.speed = self.default_speed

        elif self.direction == "back":
            self.angle = self.default_angle
            self.speed = self.backward_speed

        else:
            self.angle = self.default_angle
            self.speed = self.default_speed

    def compare_space(self, first_dst, last_dst):
        if self.obstacle_exit:
            if first_dst > last_dst and self.dist_data > self.min_dist:
                self.direction = "right"
            elif first_dst <= last_dst and self.dist_data > self.min_dist:
                self.direction = "left"
            else:
                self.direction = "back"
        else:
            self.direction = "front"

    def main(self):
        if self.is_scan:
            first, first_dst, last, last_dst = self.LiDAR_scan()
            self.compare_space(first_dst, last_dst)
            self.move_direction(last, first)

            self.cmd_vel_msg.linear.x = self.speed
            self.cmd_vel_msg.angular.z = self.angle
            self.pub.publish(self.cmd_vel_msg)

            current_mode = "obstacle"
        else:
            current_mode = "lane"

        # 이전 상태와 현재 상태를 비교하여 변경된 경우에만 퍼블리시
        if current_mode != self.previous_mode:
            self.mode_pub.publish(current_mode)
            self.previous_mode = current_mode

        self.rate.sleep()

if __name__ == "__main__":
    limo_obstacle_avoidence = Limo_obstacle_avoidence()
    try:
        while not rospy.is_shutdown():
            limo_obstacle_avoidence.main()
    except rospy.ROSInterruptException:
        pass
