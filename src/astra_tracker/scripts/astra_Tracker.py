#!/usr/bin/env python
# coding: utf-8
import os
import time
import rospy
import getpass
import threading
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from astra_common import *
from astra_tracker.cfg import AstraColorHSVConfig
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from cv_bridge import CvBridge

class Astra_Tracker:
    def __init__(self):
        rospy.init_node("Astra_Tracker", anonymous=False)
        rospy.on_shutdown(self.cancel)
        
        # 初始化变量
        self.index = 2
        self.Roi_init = ()
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.point_pose = (0, 0, 0)
        self.dyn_update = True
        self.Start_state = True
        self.select_flags = False
        self.gTracker_state = False
        self.windows_name = 'frame'
        self.color = color_follow()
        self.cols, self.rows = 0, 0
        self.Mouse_XY = (0, 0)
        self.tracker_types = ['BOOSTING', 'MIL', 'KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT', 'color']
        self.tracker_type = rospy.get_param("~tracker_type", 'color')
        self.user_name = getpass.getuser()
        text_path = '/home/' + self.user_name + '/ascam_ws/src/astra_tracker/scripts'
        self.hsv_text = text_path + "/AstraTrackerHSV.text"
        
        # 动态重新配置服务器
        Server(AstraColorHSVConfig, self.dynamic_reconfigure_callback)
        self.dyn_client = Client("/Astra_Tracker", timeout=60)

        # OpenCV 和 ROS 图像转换桥
        self.bridge = CvBridge()

        # ROS 订阅器
        self.image_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, self.image_callback)
        
        # 初始化属性
        self.Track_state = 'init'  # 确保 Track_state 已初始化
        self.image = None

        rospy.loginfo("Astra Tracker initialized.")

    def dynamic_reconfigure_callback(self, config, level):
        self.hsv_range = ((config['Hmin'], config['Smin'], config['Vmin']),
                          (config['Hmax'], config['Smax'], config['Vmax']))
        write_HSV(self.hsv_text, self.hsv_range)
        return config

    def onMouse(self, event, x, y, flags, param):
        if event == 1:
            self.Track_state = 'init'
            self.select_flags = True
            self.Mouse_XY = (x, y)
        if event == 4:
            self.select_flags = False
            self.Track_state = 'mouse'
        if self.select_flags == True:
            self.cols = min(self.Mouse_XY[0], x), min(self.Mouse_XY[1], y)
            self.rows = max(self.Mouse_XY[0], x), max(self.Mouse_XY[1], y)
            self.Roi_init = (self.cols[0], self.cols[1], self.rows[0], self.rows[1])

    def process(self, rgb_img, action):
        rgb_img = cv.resize(rgb_img, (640, 480))
        binary = []
        
        # 根据键盘输入控制状态
        if action == ord('i') or action == ord('I'): 
            self.Track_state = "identify"
        elif action == ord('r') or action == ord('R'): 
            self.Reset()
        elif action == ord('q') or action == ord('Q'): 
            self.cancel()
        elif action == ord('f') or action == ord('F'):
            self.index += 1
            if self.index >= len(self.tracker_types): 
                self.index = 0
            self.tracker_type = self.tracker_types[self.index]
            self.gTracker = Tracker(tracker_type=self.tracker_type)
            rospy.loginfo("Tracker switched to: {}".format(self.tracker_type))  # 使用 format 格式化日志
            self.Reset()
            self.Track_state = 'init'
        
        if self.Track_state == 'init':
            cv.namedWindow(self.windows_name, cv.WINDOW_AUTOSIZE)
            cv.setMouseCallback(self.windows_name, self.onMouse, 0)
            if self.select_flags == True:
                cv.line(rgb_img, self.cols, self.rows, (255, 0, 0), 2)
                cv.rectangle(rgb_img, self.cols, self.rows, (0, 255, 0), 2)
                if self.Roi_init[0] != self.Roi_init[2] and self.Roi_init[1] != self.Roi_init[3]:
                    if self.tracker_type == "color": 
                        rgb_img, self.hsv_range = self.color.Roi_hsv(rgb_img, self.Roi_init)
                    self.gTracker_state = True
                    self.dyn_update = True
                else: 
                    self.Track_state = 'init'
        elif self.Track_state == "identify":
            if os.path.exists(self.hsv_text) and self.tracker_type == "color":
                self.hsv_range = read_HSV(self.hsv_text)
            else: 
                self.Track_state = 'init'
        if self.Track_state != 'init':
            if self.tracker_type == "color" and len(self.hsv_range) != 0:
                rgb_img, binary = self.color.object_follow(rgb_img, self.hsv_range)
                if self.dyn_update == True:
                    write_HSV(self.hsv_text, self.hsv_range)
                    params = {'Hmin': self.hsv_range[0][0], 'Hmax': self.hsv_range[1][0],
                              'Smin': self.hsv_range[0][1], 'Smax': self.hsv_range[1][1],
                              'Vmin': self.hsv_range[0][2], 'Vmax': self.hsv_range[1][2]}
                    self.dyn_client.update_configuration(params)
                    self.dyn_update = False
            if self.tracker_type != "color":
                if self.gTracker_state == True:
                    Roi = (self.Roi_init[0], self.Roi_init[1], self.Roi_init[2] - self.Roi_init[0], self.Roi_init[3] - self.Roi_init[1])
                    self.gTracker = Tracker(tracker_type=self.tracker_type)
                    self.gTracker.initWorking(rgb_img, Roi)
                    self.gTracker_state = False
                rgb_img = self.gTracker.track(rgb_img)
        if self.tracker_type == "color": 
            cv.putText(rgb_img, " color", (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        else: 
            cv.putText(rgb_img, "{} Tracker".format(self.tracker_type), (260, 20), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2)
        
        return rgb_img, binary

    def cancel(self):
        self.Reset()
        self.dyn_client.close()
        cv.destroyAllWindows()
        rospy.signal_shutdown("User requested shutdown")

    def Reset(self):
        self.hsv_range = ()
        self.circle = (0, 0, 0)
        self.Mouse_XY = (0, 0)
        self.Track_state = 'init'
        rospy.loginfo("init succes!!!")

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image using CvBridge
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            action = cv.waitKey(10) & 0xFF  # Check for key press
            if action == ord('q') or action == ord('Q'):
                self.cancel()  # Trigger exit if 'q' is pressed
                return
            frame, binary = self.process(cv_image, action)
            cv.imshow('frame', frame)
        except Exception as e:
            rospy.logerr("Failed to convert image: {}".format(str(e)))  # 使用 format 格式化错误消息

if __name__ == '__main__':
    astra_tracker = Astra_Tracker()
    rospy.spin()  # Keep the ROS node running

