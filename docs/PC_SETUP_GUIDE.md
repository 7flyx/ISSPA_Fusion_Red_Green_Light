# PC端安装指南
# PC Setup Guide

## 为什么需要安装？

PC端需要运行YOLOv5检测节点，所以需要：
1. 项目代码（特别是`yolov5_ros`包）
2. 编译工作空间
3. 安装Python依赖

## 快速安装步骤

### 步骤1: 克隆项目（最小化）

```bash
# 创建ROS工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 只克隆必要的包（如果使用git）
# 或者直接从Jetson复制必要的包
```

### 步骤2: 从Jetson复制必要的包

在Jetson上：

```bash
# 打包必要的包
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src
tar -czf ~/isspa_pc_packages.tar.gz \
    isspa_object_detection/camera_based_detection/yolov5_ros \
    isspa_navigation/navigation_stack/launch/pavs_navigation_pc_only.launch \
    isspa_navigation/navigation_stack/launch/pavs_navigation_vehicle_only.launch

# 传输到PC（使用scp或其他方式）
scp ~/isspa_pc_packages.tar.gz flyx@<PC_IP>:~/Desktop/
```

在PC上：

```bash
# 解压到工作空间
cd ~/catkin_ws/src
tar -xzf ~/Desktop/isspa_pc_packages.tar.gz

# 或者手动创建目录结构
mkdir -p ~/catkin_ws/src/isspa_object_detection/camera_based_detection
mkdir -p ~/catkin_ws/src/isspa_navigation/navigation_stack/launch
```

### 步骤3: 安装依赖

```bash
# 安装ROS依赖
sudo apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-move-base-msgs \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-topic-tools \
    ros-noetic-image-view

# 安装Python依赖
pip3 install ultralytics opencv-python numpy torch torchvision
```

### 步骤4: 编译工作空间

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 步骤5: 测试

```bash
# 检查包是否存在
rospack find yolov5_ros

# 检查launch文件
roslaunch navigation_stack pavs_navigation_pc_only.launch --check
```

## 最小化安装（只复制必要文件）

如果不想复制整个项目，可以只复制必要的文件：

### 在Jetson上创建最小化包

```bash
# 创建最小化包结构
mkdir -p ~/isspa_pc_minimal/src/isspa_object_detection/camera_based_detection
mkdir -p ~/isspa_pc_minimal/src/isspa_navigation/navigation_stack/launch

# 复制yolov5_ros包
cp -r ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_object_detection/camera_based_detection/yolov5_ros \
    ~/isspa_pc_minimal/src/isspa_object_detection/camera_based_detection/

# 复制launch文件
cp ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_navigation/navigation_stack/launch/pavs_navigation_pc_only.launch \
    ~/isspa_pc_minimal/src/isspa_navigation/navigation_stack/launch/
cp ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_navigation/navigation_stack/launch/pavs_navigation_vehicle_only.launch \
    ~/isspa_pc_minimal/src/isspa_navigation/navigation_stack/launch/

# 打包
cd ~/isspa_pc_minimal
tar -czf ~/isspa_pc_minimal.tar.gz src/
```

### 在PC上解压和编译

```bash
# 解压到工作空间
cd ~/catkin_ws
tar -xzf ~/Desktop/isspa_pc_minimal.tar.gz

# 编译
catkin_make
source devel/setup.bash
```

## 完整克隆（推荐）

如果你想在PC上完整开发，可以克隆整个项目：

```bash
# 在PC上
cd ~/catkin_ws/src
git clone <your-repo-url> ISSPA

# 或者从Jetson同步
rsync -avz jetson@<JETSON_IP>:~/ISSPA2/ISSPA_Fusion_Red_Green_Light/ \
    ~/catkin_ws/src/ISSPA/

# 编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 验证安装

```bash
# 1. 检查包
rospack find yolov5_ros

# 2. 检查launch文件
roslaunch navigation_stack pavs_navigation_pc_only.launch --check

# 3. 检查Python脚本
ls -l $(rospack find yolov5_ros)/scripts/*.py

# 4. 检查模型文件（如果有）
ls -l $(rospack find yolov5_ros)/scripts/yolov5*.pt
```

## 常见问题

### 问题1: 找不到包

```bash
# 确保已source
source ~/catkin_ws/devel/setup.bash

# 检查ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH
```

### 问题2: 找不到消息类型

```bash
# 确保所有依赖包都已编译
cd ~/catkin_ws
catkin_make
```

### 问题3: Python模块缺失

```bash
# 安装所有依赖
pip3 install -r $(rospack find yolov5_ros)/scripts/requirements.txt
```

