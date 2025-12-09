# Jetson Orin Nano 依赖安装指南
# Jetson Orin Nano Dependencies Installation Guide

## 错误修复 / Error Fixes

### 错误1: ImportError: cannot import name 'GoalID'

**问题**: `GoalID` 导入错误

**解决**: 已修复，`GoalID` 应该从 `actionlib_msgs.msg` 导入

### 错误2: ModuleNotFoundError: No module named 'ultralytics'

**问题**: 缺少 `ultralytics` 模块

**解决方案**:

#### 方案A: 安装ultralytics（推荐）

```bash
pip3 install ultralytics
```

#### 方案B: 使用本地utils（如果不想安装ultralytics）

代码已修改为自动fallback到本地utils，但如果仍有问题，可以：

```bash
# 检查是否有本地的plotting模块
ls src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts/utils/plots.py
```

## 完整依赖安装 / Complete Dependencies Installation

### 1. 安装Python依赖

```bash
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts

# 安装所有依赖
pip3 install -r requirements.txt

# 如果遇到问题，单独安装关键依赖
pip3 install ultralytics
pip3 install opencv-python
pip3 install pillow
pip3 install numpy
pip3 install PyYAML
```

### 2. 安装ROS依赖

```bash
# 安装actionlib（包含GoalID）
sudo apt-get install -y ros-noetic-actionlib ros-noetic-actionlib-msgs

# 安装其他ROS依赖
sudo apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-move-base-msgs
```

### 3. 重新编译工作空间

```bash
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light
catkin_make -j1  # 使用单线程避免内存不足
source devel/setup.bash
```

## 快速修复命令 / Quick Fix Commands

```bash
# 1. 安装ultralytics
pip3 install ultralytics

# 2. 安装ROS actionlib
sudo apt-get install -y ros-noetic-actionlib ros-noetic-actionlib-msgs

# 3. 重新编译
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light
catkin_make -j1
source devel/setup.bash

# 4. 重新启动
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    enable_camera:=true \
    enable_visualization:=true
```

## 验证安装 / Verification

### 检查ultralytics

```bash
python3 -c "import ultralytics; print('ultralytics installed')"
```

### 检查ROS消息

```bash
python3 << EOF
import rospy
from actionlib_msgs.msg import GoalID
print("GoalID import successful")
EOF
```

## 如果仍然有问题 / If Still Having Issues

### 检查Python路径

```bash
# 检查Python版本
python3 --version

# 检查pip3
pip3 --version

# 检查安装位置
pip3 show ultralytics
```

### 检查ROS环境

```bash
# 检查ROS版本
echo $ROS_DISTRO

# 检查ROS包
rospack find actionlib_msgs
```

