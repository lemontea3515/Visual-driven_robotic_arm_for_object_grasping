#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO

class YOLOv8Detector:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('yolov8_rgb_depth_detector_no_caminfo', anonymous=True)
        
        # 加载 YOLOv8 模型（可根据需求更换，如 yolov8s.pt）
        self.model = YOLO('yolov8s.pt')
        rospy.loginfo("YOLOv8 模型加载完成")

        # 初始化 CvBridge（用于 ROS 图像与 OpenCV 图像转换）
        self.bridge = CvBridge()

        # 定义发布者（处理后的检测图像）
        self.detected_pub = rospy.Publisher('/yolov8/detected_image', Image, queue_size=1)

        # 订阅 RGB 和深度图像话题（不再订阅 camera_info）
        rgb_sub = Subscriber('/ascamera_hp60c/rgb0/image', Image)        # RGB 图像话题
        depth_sub = Subscriber('/ascamera_hp60c/depth0/image_raw', Image)    # 深度图像话题

        # 时间同步器（仅同步 RGB 和深度图像，允许 0.1 秒时间差）
        ts = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1  # 时间差容限（秒）
        )
        ts.registerCallback(self.sync_callback)

    def sync_callback(self, rgb_msg, depth_msg):
        """同步回调函数（仅处理 RGB 和深度图像）"""
        try:
            # --------------------------
            # 步骤 1：转换图像格式
            # --------------------------
            # 转换 RGB 图像（BGR 格式，OpenCV 默认）
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
            
            # 转换深度图像（16UC1 → 米制浮点，单位：米）
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '16UC1')
            depth_image = depth_image.astype(np.float32) / 1000.0  # 毫米 → 米

            # --------------------------
            # 步骤 2：验证图像尺寸匹配（关键！）
            # --------------------------
            rgb_height, rgb_width = rgb_image.shape[1],rgb_image.shape[0]  # RGB 图像尺寸（高, 宽）
            depth_height, depth_width = depth_image.shape[1],depth_image.shape[0]  # 深度图像尺寸（高, 宽）
            
            # 检查 RGB 与深度图像尺寸是否一致
            if (rgb_width, rgb_height) != (depth_width, depth_height):
                rospy.logwarn(f"图像尺寸不匹配: RGB({rgb_width}x{rgb_height}) vs 深度({depth_width}x{depth_height})")
                # 可选：跳过本次检测（或调整尺寸）
                return

            # --------------------------
            # 步骤 3：执行 YOLOv8 目标检测
            # --------------------------
            results = self.model(rgb_image, verbose=False)  # 关闭模型自带的输出

            # --------------------------
            # 步骤 4：处理检测结果并叠加可视化
            # --------------------------
            annotated_img = self.process_detections(results[0], rgb_image, depth_image)

            # --------------------------
            # 步骤 5：发布处理后的图像
            # --------------------------
            self.publish_detected_image(annotated_img)

        except Exception as e:
            rospy.logerr(f"处理异常: {str(e)}")

    def process_detections(self, result, rgb_image, depth_image):
        """处理检测结果并叠加可视化信息（无相机参数依赖）"""
        annotated_img = rgb_image.copy()
        boxes = result.boxes.xyxy.cpu().numpy()  # 边界框坐标（xyxy 格式）
        confidences = result.boxes.conf.cpu().numpy()  # 置信度
        class_ids = result.boxes.cls.cpu().numpy().astype(int)  # 类别 ID
        
        # 获取类别名称字典（YOLOv8 内置）
        class_names = self.model.names

        for box, conf, cls_id in zip(boxes, confidences, class_ids):
            x1, y1, x2, y2 = map(int, box)
            
            # 过滤无效边界框（超出图像范围）
            if x1 < 0 or x2 > annotated_img.shape[1] or y1 < 0 or y2 > annotated_img.shape[0]:
                continue

            # 获取类别名称
            class_name = class_names.get(cls_id, f'Class_{cls_id}')
            
            # --------------------------
            # 计算边界框内的平均深度（关键！）
            # --------------------------
            # 提取边界框区域（注意：图像坐标系原点在左上角）
            roi_depth = depth_image[y1:y2, x1:x2]
            
            # 过滤无效深度值（根据传感器特性调整范围，如 0.1m~10m）
            valid_depth = roi_depth[(roi_depth > 0.1) & (roi_depth < 10.0)]
            
            # 计算平均深度（若有效像素不足则标记为 -1）
            avg_depth = np.mean(valid_depth) if valid_depth.size > 0 else -1.0

            # --------------------------
            # 构建并绘制标签
            # --------------------------
            label = f"{class_name} {conf:.2f}"
            if avg_depth > 0:
                label += f" | {avg_depth:.2f}m"

            # 绘制边界框（绿色，线宽 2）
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 绘制标签背景（与边界框同色）
            (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(annotated_img, 
                         (x1, y1 - text_height - 10), 
                         (x1 + text_width, y1), 
                         (0, 255, 0), -1)  # -1 表示填充

            # 绘制标签文本（黑色字体）
            cv2.putText(annotated_img, label, 
                       (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                       (0, 0, 0), 2)  # 黑色字体，线宽 2

        return annotated_img

    def publish_detected_image(self, image):
        """发布处理后的检测图像"""
        try:
            # 转换 OpenCV 图像为 ROS 消息（BGR8 格式）
            detected_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            detected_msg.header.stamp = rospy.Time.now()  # 同步时间戳
            self.detected_pub.publish(detected_msg)
        except CvBridgeError as e:
            rospy.logerr(f"图像转换失败: {str(e)}")

if __name__ == '__main__':
    detector = YOLOv8Detector()
    rospy.spin()
