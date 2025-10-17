#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from ultralytics import YOLO

# 添加调试信息
print("🚀 脚本开始执行...")

class YOLOv8OnDemandDetector:
    def __init__(self):
        rospy.init_node('yolov8_on_demand_detector', anonymous=True)

        # ======================
        # 1. 加载 YOLOv8 模型（推荐 yolov8n.pt，轻量快速）
        # ======================
        print("开始加载 YOLOv8 模型...")
        self.model = YOLO('yolov8n.pt')  # 你也可以使用 yolov8s.pt / yolov8m.pt
        rospy.loginfo("✅ YOLOv8 模型 (yolov8n.pt) 加载完成")

        # ======================
        # 2. 初始化 CvBridge
        # ======================
        self.bridge = CvBridge()

        # ======================
        # 3. 缓存最新的 RGB 和深度图
        # ======================
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.rgb_ready = False
        self.depth_ready = False

        # ======================
        # 4. 相机内参（非常重要，请根据你的实际相机修改！）
        # ======================
        self.fx = 578.52  # 焦距 fx [像素]
        self.fy = 578.05  # 焦距 fy [像素]
        self.cx = 336.00  # 光心 cx [像素]
        self.cy = 239.23  # 光心 cy [像素]

        # ======================
        # 5. 订阅 RGB 图像
        # ======================
        self.rgb_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, self.rgb_callback)

        # ======================
        # 6. 订阅深度图
        # ======================
        self.depth_sub = rospy.Subscriber('/ascamera_hp60c/depth0/image_raw', Image, self.depth_callback)

        # ======================
        # 7. 订阅检测触发信号（比如一个空消息）：/detect
        # ======================
        self.detect_sub = rospy.Subscriber('/detect', Empty, self.detect_callback)

        rospy.loginfo("🟢 节点初始化完成，等待图像数据和 /detect 触发信号...")

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.rgb_ready = True
            # 不做任何处理，只缓存
        except Exception as e:
            rospy.logerr(f"RGB 图像转换失败: {e}")

    def depth_callback(self, msg):
        try:
            # 深度图通常是 16UC1，单位是毫米，需要转为米
            depth_image_raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.latest_depth_image = depth_image_raw.astype(np.float32) / 1000.0  # 转成米
            self.depth_ready = True
            # 不做任何处理，只缓存
        except Exception as e:
            rospy.logerr(f"深度图转换失败: {e}")

    def detect_callback(self, msg):
        # 检查是否已收到有效的 RGB 和深度图
        if not self.rgb_ready or not self.depth_ready:
            rospy.logwarn("⚠️ 未收到完整的 RGB + 深度图，无法检测！请确保相机驱动正常运行。")
            return

        if self.latest_rgb_image is None or self.latest_depth_image is None:
            rospy.logwarn("⚠️ 最新图像数据为空！")
            return

        if self.latest_rgb_image.shape[:2] != self.latest_depth_image.shape[:2]:
            rospy.logwarn(f"❌ RGB尺寸 {self.latest_rgb_image.shape[:2]} 与深度图 {self.latest_depth_image.shape[:2]} 不匹配！")
            return

        rospy.loginfo("🚀 收到 /detect 信号，开始对当前帧进行单次检测...")

        try:
            rgb_image = self.latest_rgb_image
            depth_image = self.latest_depth_image  # 单位：米
            height, width = rgb_image.shape[:2]

            # =========================
            # 1. 可选：缩小图像以加速检测（推荐！）
            # =========================
            scale_percent = 50  # 缩放比例，比如 50%
            width_small = int(width * scale_percent / 100)
            height_small = int(height * scale_percent / 100)
            dim = (width_small, height_small)
            small_img = cv2.resize(rgb_image, dim, interpolation=cv2.INTER_LINEAR)

            # =========================
            # 2. 使用 YOLOv8 检测
            # =========================
            results = self.model(small_img, verbose=False)
            boxes = results[0].boxes.xyxy.cpu().numpy()         # [x1,y1,x2,y2]（小图坐标）
            confidences = results[0].boxes.conf.cpu().numpy()   # 置信度
            class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
            class_names = results[0].names

            # =========================
            # 3. 遍历每个检测框，计算 3D 坐标
            # =========================
            for i, (box, conf, cls_id) in enumerate(zip(boxes, confidences, class_ids)):
                x1_small, y1_small, x2_small, y2_small = map(int, box)

                if x1_small < 0 or x2_small > width_small or y1_small < 0 or y2_small > height_small:
                    continue

                class_name = class_names.get(cls_id, f"Class_{cls_id}")
                confidence = conf

                # 映射回原图坐标
                x1_orig = int(x1_small * width / width_small)
                y1_orig = int(y1_small * height / height_small)
                x2_orig = int(x2_small * width / width_small)
                y2_orig = int(y2_small * height / height_small)

                cx_orig = int((x1_orig + x2_orig) / 2)
                cy_orig = int((y1_orig + y2_orig) / 2)

                # 从深度图获取 Z（单位：米）
                Z = depth_image[cy_orig, cx_orig]

                if Z <= 0 or Z > 10.0:  # 合理范围检查
                    rospy.logdebug(f"⚠️ 检测框中心 ({cx_orig}, {cy_orig}) 深度值无效: {Z}")
                    continue

                # 相机坐标系下的 3D 坐标
                X = (cx_orig - self.cx) * Z / self.fx
                Y = (cy_orig - self.cy) * Z / self.fy
                Z = Z  # 单位：米

                # 打印结果
                rospy.loginfo(
                    f"🔍 物体 {i+1}: 类别={class_name}, 置信度={confidence:.2f}, "
                    f"3D坐标=(X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}) [米]"
                )

        except Exception as e:
            rospy.logerr(f"检测过程中出现异常: {e}")

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    detector = YOLOv8OnDemandDetector()
    detector.run()
