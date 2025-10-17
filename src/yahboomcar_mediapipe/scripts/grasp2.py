#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import json
import serial
import threading
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from ultralytics import YOLO

class YOLOv8ArmController:
    def __init__(self):
        rospy.init_node('yolov8_arm_controller', anonymous=True)

        # ======================
        # 1. 从ROS参数服务器获取串口配置
        # ======================
        serial_port = rospy.get_param('~serial_port', '/dev/ttyUSB0')
        baudrate = rospy.get_param('~baudrate', 115200)
        
        rospy.loginfo(f"连接串口: {serial_port}, 波特率: {baudrate}")
        
        try:
            self.ser = serial.Serial(serial_port, baudrate=baudrate, dsrdtr=None)
            self.ser.setRTS(False)
            self.ser.setDTR(False)
            
            # 启动串口读取线程
            self.serial_recv_thread = threading.Thread(target=self.read_serial)
            self.serial_recv_thread.daemon = True
            self.serial_recv_thread.start()
            
        except Exception as e:
            rospy.logerr(f"串口连接失败: {e}")
            self.ser = None

        # ======================
        # 2. 机械臂坐标限位 (单位: mm)
        # ======================
        self.x_limits = (5, 475)      # x范围：5～475
        self.y_limits = (-250, 250)   # y范围：-250～250  
        self.z_limits = (-105, 400)      # z范围：0～400
        
        # ======================
        # 3. 夹爪控制参数
        # ======================
        self.gripper_open_angle = 2.3   # 夹爪张开角度
        self.gripper_close_angle = 3.14  # 夹爪闭合角度
        self.gripper_speed = 0          # 夹爪运动速度
        self.gripper_acc = 0            # 夹爪加速度
        
        # 抓取偏移距离 (cm转换为mm)
        self.grasp_offset = 30  # 3cm = 30mm

        # ======================
        # 4. 机械臂初始化位置
        # ======================
        self.arm_init_command = {
            "T": 102,
            "base": -0.01,          # 底座关节
            "shoulder": -0.00,  # 肩部关节
            "elbow": 1.59,      # 肘部关节
            "hand": 3.14,       # 手部关节
            "spd": 1.2,          # 速度
            "acc": 1.2           # 加速度
        }

        # ======================
        # 5. 加载 YOLOv8 模型
        # ======================
        print("开始加载 YOLOv8 模型...")
        self.model = YOLO('yolov8n.pt')
        rospy.loginfo("YOLOv8 模型 (yolov8n.pt) 加载完成")

        # ======================
        # 6. 初始化 CvBridge
        # ======================
        self.bridge = CvBridge()

        # ======================
        # 7. 缓存最新的 RGB 和深度图
        # ======================
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.rgb_ready = False
        self.depth_ready = False

        # ======================
        # 8. 相机内参
        # ======================
        self.fx = 578.52  # 焦距 fx [像素]
        self.fy = 578.05  # 焦距 fy [像素]
        self.cx = 336.00  # 光心 cx [像素]
        self.cy = 239.23  # 光心 cy [像素]

        # ======================
        # 9. 坐标偏移参数 (单位: 米)
        # ======================
        self.x_offset = -0.10    
        self.y_offset = -0.04    
        self.z_offset = -0.011   

        # ======================
        # 10. 订阅图像和检测触发信号
        # ======================
        self.rgb_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/ascamera_hp60c/depth0/image_raw', Image, self.depth_callback)
        self.detect_sub = rospy.Subscriber('/detect', Empty, self.detect_callback)

        rospy.loginfo(f"节点初始化完成，串口: {serial_port}")

        # ======================
        # 11. 程序启动时初始化机械臂
        # ======================
        rospy.loginfo("程序启动，正在初始化机械臂...")
        # 等待一段时间确保ROS连接稳定
        rospy.sleep(2.0)
        self.initialize_arm_position()

    def initialize_arm_position(self):
        """
        发送机械臂初始化位置指令
        {"T":102,"base":0,"shoulder":,"elbow":2.46,"hand":3.14,"spd":10,"acc":10}
        """
        if self.ser is None:
            rospy.logwarn("串口未连接，无法发送初始化指令")
            return False
            
        command_str = json.dumps(self.arm_init_command) + '\n'
        try:
            self.ser.write(command_str.encode())
            rospy.loginfo(f"发送机械臂初始化指令: {command_str.strip()}")
            
            # 等待机械臂运动到初始化位置
            rospy.sleep(3.0)
            rospy.loginfo("机械臂初始化完成")
            return True
            
        except Exception as e:
            rospy.logerr(f"初始化指令发送错误: {e}")
            return False

    def check_arm_limits(self, x, y, z):
        """
        检查机械臂坐标是否在限位范围内
        返回: (是否有效, 错误信息)
        """
        if not (self.x_limits[0] <= x <= self.x_limits[1]):
            return False, f"X坐标{x:.1f}超出范围[{self.x_limits[0]}, {self.x_limits[1]}]"
        
        if not (self.y_limits[0] <= y <= self.y_limits[1]):
            return False, f"Y坐标{y:.1f}超出范围[{self.y_limits[0]}, {self.y_limits[1]}]"
        
        if not (self.z_limits[0] <= z <= self.z_limits[1]):
            return False, f"Z坐标{z:.1f}超出范围[{self.z_limits[0]}, {self.z_limits[1]}]"
        
        return True, "坐标在限位范围内"

    def send_arm_command(self, x, y, z, t=3.14, spd=0.25, command_type=104):
        """
        发送JSON指令到机械臂
        {"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}
        """
        if self.ser is None:
            rospy.logwarn("串口未连接，无法发送指令")
            return False
            
        # 检查坐标限位
        is_valid, error_msg = self.check_arm_limits(x, y, z)
        if not is_valid:
            rospy.logerr(f"坐标超限: {error_msg}")
            return False
            
        command = {
            "T": command_type,
            "x": round(x, 3),
            "y": round(y, 3),
            "z": round(z, 3),
            "t": round(t, 3),
            "spd": round(spd, 3)
        }
        
        command_str = json.dumps(command) + '\n'
        try:
            self.ser.write(command_str.encode())
            rospy.loginfo(f"发送指令到机械臂: {command_str.strip()}")
            return True
        except Exception as e:
            rospy.logerr(f"串口发送错误: {e}")
            return False

    def send_gripper_command(self, angle, spd=None, acc=None):
        """
        发送夹爪控制指令
        {"T":106,"cmd":3.14,"spd":10,"acc":10}
        """
        if self.ser is None:
            rospy.logwarn("串口未连接，无法发送夹爪指令")
            return False
            
        # 限制夹爪角度在范围内
        angle = max(self.gripper_open_angle, min(angle, self.gripper_close_angle))
        
        command = {
            "T": 106,
            "cmd": round(angle, 3),
            "spd": spd if spd is not None else self.gripper_speed,
            "acc": acc if acc is not None else self.gripper_acc
        }
        
        command_str = json.dumps(command) + '\n'
        try:
            self.ser.write(command_str.encode())
            gripper_state = "张开" if angle <= 1.5 else "闭合" if angle >= 2.8 else "中间状态"
            rospy.loginfo(f"发送夹爪指令: 角度={angle:.2f} ({gripper_state})")
            return True
        except Exception as e:
            rospy.logerr(f"夹爪指令发送错误: {e}")
            return False

    def execute_grasp_sequence(self, target_x, target_y, target_z):
        """
        执行完整的抓取序列
        1. 初始化机械臂位置
        2. 运动到目标位置上方3cm处
        3. 打开夹爪
        4. 前进3cm到目标位置
        5. 闭合夹爪
        """
        rospy.loginfo("开始执行抓取序列...")
        
        """
        # 1. 初始化机械臂位置
        rospy.loginfo("1. 初始化机械臂位置")
        if not self.initialize_arm_position():
            rospy.logerr("机械臂初始化失败，中止抓取")
            return False
            """
        
        # 1. 运动到目标位置上方3cm处
        approach_z = target_z + self.grasp_offset  # 在Z轴上方3cm
        is_valid, error_msg = self.check_arm_limits(target_x, target_y, approach_z)
        if not is_valid:
            rospy.logerr(f"接近位置超限: {error_msg}")
            return False
            
        rospy.loginfo(f"1. 运动到接近位置: X={target_x:.1f}, Y={target_y:.1f}, Z={approach_z:.1f}")
        if not self.send_arm_command(target_x, target_y, approach_z, spd=0.15):
            return False
            
        # 等待机械臂运动完成
        rospy.sleep(5.0)
        
        # 2. 打开夹爪
        rospy.loginfo("2. 打开夹爪")
        if not self.send_gripper_command(self.gripper_open_angle):
            return False
            
        rospy.sleep(3.0)
        
        # 3. 前进到目标位置
        rospy.loginfo(f"3. 前进到目标位置: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}")
        if not self.send_arm_command(target_x, target_y, target_z, spd=0.1):
            return False
            
        rospy.sleep(3.0)
        
        # 4. 闭合夹爪
        rospy.loginfo("4. 闭合夹爪")
        if not self.send_gripper_command(self.gripper_close_angle):
            return False
            
        rospy.sleep(1.0)
        rospy.loginfo("抓取序列完成")
        
        # 5. 初始化机械臂位置
        rospy.loginfo("5. 初始化机械臂位置")
        if not self.initialize_arm_position():
            rospy.logerr("机械臂初始化失败")
            return False

        return True

    def read_serial(self):
        """读取串口数据的线程函数"""
        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.readline().decode('utf-8').strip()
                    if data:
                        rospy.loginfo(f"机械臂响应: {data}")
            except Exception as e:
                rospy.logerr(f"串口读取错误: {e}")

    def camera_to_arm_coordinates(self, X_camera, Y_camera, Z_camera):
        """
        将相机坐标系下的点转换到机械臂坐标系
        
        参数:
            X_camera, Y_camera, Z_camera: 相机坐标系下的坐标 (单位: 米)
        返回:
            [x, y, z]: 机械臂坐标系下的坐标 (单位: 毫米)
        """
        # 应用偏移量 (单位: 米)
        X_arm = -X_camera + 0.30972 + self.x_offset
        Y_arm = -Y_camera - 0.00285 + self.y_offset
        Z_arm = -Z_camera + 0.23127 + self.z_offset  
        
        # 转换为毫米
        x_mm = X_arm * 1000
        y_mm = Y_arm * 1000
        z_mm = Z_arm * 1000
        
        return [x_mm, y_mm, z_mm]

    def rgb_callback(self, msg):
        try:
            self.latest_rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.rgb_ready = True
        except Exception as e:
            rospy.logerr(f"RGB 图像转换失败: {e}")

    def depth_callback(self, msg):
        try:
            depth_image_raw = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.latest_depth_image = depth_image_raw.astype(np.float32) / 1000.0
            self.depth_ready = True
        except Exception as e:
            rospy.logerr(f"深度图转换失败: {e}")

    def detect_callback(self, msg):
        """检测回调函数，找到置信度最高的物体并发送给机械臂"""
        if not self.rgb_ready or not self.depth_ready:
            rospy.logwarn("⚠️ 未收到完整的 RGB + 深度图，无法检测！")
            return

        if self.latest_rgb_image is None or self.latest_depth_image is None:
            rospy.logwarn("⚠️ 最新图像数据为空！")
            return

        rospy.loginfo("🚀 收到 /detect 信号，开始检测并执行抓取...")

        try:
            rgb_image = self.latest_rgb_image
            depth_image = self.latest_depth_image
            height, width = rgb_image.shape[:2]

            # 缩小图像以加速检测
            scale_percent = 50
            width_small = int(width * scale_percent / 100)
            height_small = int(height * scale_percent / 100)
            small_img = cv2.resize(rgb_image, (width_small, height_small), interpolation=cv2.INTER_LINEAR)

            # YOLOv8 检测
            results = self.model(small_img, verbose=False)
            boxes = results[0].boxes.xyxy.cpu().numpy()
            confidences = results[0].boxes.conf.cpu().numpy()
            class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
            class_names = results[0].names

            if len(boxes) == 0:
                rospy.logwarn("❌ 未检测到任何物体")
                return

            # 找到置信度最高的检测结果
            best_idx = np.argmax(confidences)
            best_box = boxes[best_idx]
            best_conf = confidences[best_idx]
            best_class_id = class_ids[best_idx]
            best_class_name = class_names.get(best_class_id, f"Class_{best_class_id}")

            # 映射回原图坐标
            x1_small, y1_small, x2_small, y2_small = map(int, best_box)
            x1_orig = int(x1_small * width / width_small)
            y1_orig = int(y1_small * height / height_small)
            x2_orig = int(x2_small * width / width_small)
            y2_orig = int(y2_small * height / height_small)

            cx_orig = int((x1_orig + x2_orig) / 2)
            cy_orig = int((y1_orig + y2_orig) / 2)

            # 从深度图获取 Z（单位：米）
            Z = depth_image[cy_orig, cx_orig]

            if Z <= 0 or Z > 10.0:
                rospy.logwarn(f"⚠️ 检测框中心 ({cx_orig}, {cy_orig}) 深度值无效: {Z}")
                return

            # 相机坐标系下的 3D 坐标
            X_camera = (cx_orig - self.cx) * Z / self.fx
            Y_camera = (cy_orig - self.cy) * Z / self.fy
            Z_camera = Z

            rospy.loginfo(
                f"最高置信度物体: {best_class_name}, 置信度={best_conf:.2f}, "
                f"相机坐标系坐标=(X={X_camera:.3f}, Y={Y_camera:.3f}, Z={Z_camera:.3f}) [米]"
            )

            # 新的坐标系转换到机械臂坐标系
            arm_coords = self.camera_to_arm_coordinates(X_camera, Y_camera, Z_camera)
            target_x, target_y, target_z = arm_coords
            
            rospy.loginfo(
                f"机械臂坐标系坐标=(X={target_x:.1f}, Y={target_y:.1f}, Z={target_z:.1f}) [毫米]"
            )

            # 检查坐标限位
            is_valid, error_msg = self.check_arm_limits(target_x, target_y, target_z)
            if not is_valid:
                rospy.logerr(f"目标位置超限: {error_msg}")
                return

            # 执行抓取序列（包含初始化）
            self.execute_grasp_sequence(target_x, target_y, target_z)

        except Exception as e:
            rospy.logerr(f"检测过程中出现异常: {e}")

    def run(self):
        """主运行函数"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("程序被用户中断")
        finally:
            if hasattr(self, 'ser') and self.ser and self.ser.is_open:
                self.ser.close()
                rospy.loginfo("串口已关闭")

if __name__ == '__main__':
    controller = YOLOv8ArmController()
    controller.run()
