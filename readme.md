## 视觉驱动机械臂自主抓取

### 软件环境：

Ubuntu20.04 + ROS-Noetic + OpenCV4.2 + Python3.8

### 硬件环境：

机械臂RoArm-M2-S + 相机Nuwa-HP60C

### 核心文件说明：

home/rm/ascam_ws/src/yahboomcar_mediapipe/scripts文件夹下：

00_demo.py：物体识别与坐标检测；

grasp1.py：以眼在手外的方式实现机械臂自主抓取物体；

grasp2.py：以眼在手上的方式实现机械臂自主抓取物体。

### 环境搭建与程序运行步骤：

###### 相机环境搭建

步骤详见“环境搭建”pdf文件；

###### 安装ROS依赖:

```
sudo apt install ros-$ROS-DISTRO-cv-bridge ros-$ROS-DISTRO-sensor-msgs ros-$ROS-DISTRO-std-msgs
```

###### 虚拟环境中安装python第三方依赖：

```
pip install opencv-python numpy pyserial ultralytics rospkg
```

###### 程序运行步骤：

1. 启动roscore

2. 启动相机

   ```
   roslaunch ascamera hp60c.launch 
   ```

3. 打开新终端，激活已经建立好的Python3.8虚拟环境（以ros_venv虚拟环境名为例）

   ```
   source ~/ros_venv/bin/activate
   ```

4. 为虚拟环境设置环境变量分配内存

   ```
   export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
   ```

5. 运行视觉抓取程序

   ```
   python3 grasp2.py --port /dev/ttyUSB0
   ```

6. 订阅检测触发信号

   ```
   rostopic pub /detect std_msgs/Empty "{}" -1
   ```

   只有检测到触发信号相机才会开始探测物体，每次触发将持续识别3秒钟，如图：

   ![bd2f9b532ce7986b6bebb9e5eacc1aad](C:\Users\realman\AppData\Roaming\DingTalk\40015735017_v2\resource_cache\bd\bd2f9b532ce7986b6bebb9e5eacc1aad.jpg)

7. 演示结果

​	下图展示了物体识别与坐标检测程序运行的结果：

![51eb9b1b65047026a9501b6e1958bee3](C:\Users\realman\AppData\Roaming\DingTalk\40015735017_v2\resource_cache\51\51eb9b1b65047026a9501b6e1958bee3.jpg)

​	下图展示了眼在手上机械臂自主抓取程序运行的完整的流程：

![84e6dfc64753a14fcef70eb8d1b153b8](C:\Users\realman\AppData\Roaming\DingTalk\40015735017_v2\resource_cache\84\84e6dfc64753a14fcef70eb8d1b153b8.jpg)