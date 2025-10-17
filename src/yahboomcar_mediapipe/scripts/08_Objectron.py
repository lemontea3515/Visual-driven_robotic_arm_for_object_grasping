#!/usr/bin/env python3
# encoding: utf-8
import mediapipe as mp
import cv2 as cv
import time
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Objectron:
    def __init__(self, staticMode=False, maxObjects=5, minDetectionCon=0.5, minTrackingCon=0.99):
        self.staticMode=staticMode
        self.maxObjects=maxObjects
        self.minDetectionCon=minDetectionCon
        self.minTrackingCon=minTrackingCon
        self.index=0
        self.modelNames = ['Shoe', 'Chair', 'Cup', 'Camera']
        self.mpObjectron = mp.solutions.objectron
        self.mpDraw = mp.solutions.drawing_utils
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])

    def findObjectron(self, frame):
        cv.putText(frame, self.modelNames[self.index], (int(frame.shape[1] / 2) - 30, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
        img_RGB = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        results = self.mpobjectron.process(img_RGB)
        if results.detected_objects:
            for id, detection in enumerate(results.detected_objects):
                self.mpDraw.draw_landmarks(frame, detection.landmarks_2d, self.mpObjectron.BOX_CONNECTIONS)
                self.mpDraw.draw_axis(frame, detection.rotation, detection.translation)
        return frame

    def configUP(self):
        self.index += 1
        if self.index>=4:self.index=0
        self.mpobjectron = self.mpObjectron.Objectron(
            self.staticMode, self.maxObjects, self.minDetectionCon, self.minTrackingCon, self.modelNames[self.index])
        
class ObjectronNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('objectron_node', anonymous=True)
        
        # 创建一个CvBridge实例
        self.bridge = CvBridge()

        # 初始化Objectron类
        self.objectron = Objectron()

        # 初始化FPS计数器
        self.pTime = 0

        # 订阅/ascamera_hp60c/rgb0/image话题
        self.image_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, self.image_callback)

        # 发布处理后的图像
        self.pub_image = rospy.Publisher('/objectron_processed_image', Image, queue_size=1)

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            return

        # 检查按键事件
        action = cv.waitKey(1) & 0xFF
        if action == ord('q'):
            rospy.signal_shutdown("User requested shutdown")
        elif action == ord('f') or action == ord('F'):
            self.objectron.configUP()

        # 处理图像
        frame = self.objectron.findObjectron(frame)

        # 计算FPS
        cTime = time.time()
        fps = 1 / (cTime - self.pTime)
        self.pTime = cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 显示结果
        cv.imshow('frame', frame)

        # 发布处理后的图像
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

    def shutdown_hook(self):
        rospy.loginfo("Shutting down objectron node.")
        cv.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = ObjectronNode()
        rospy.on_shutdown(node.shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
