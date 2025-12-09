# PC端快速安装指南
# PC Quick Installation Guide

## 方法1: 使用脚本（推荐）

### 在Jetson上

```bash
# 1. 创建最小化安装包
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light
chmod +x scripts/create_pc_minimal_package.sh
./scripts/create_pc_minimal_package.sh

# 2. 传输到PC
scp ~/isspa_pc_minimal.tar.gz flyx@<PC_IP>:~/Desktop/
```

### 在PC上

```bash
# 1. 安装最小化包
chmod +x scripts/install_pc_minimal.sh
./scripts/install_pc_minimal.sh

# 2. Source工作空间
source ~/catkin_ws/devel/setup.bash

# 3. 测试
rospack find yolov5_ros
roslaunch navigation_stack pavs_navigation_pc_only.launch --check
```

## 方法2: 手动安装

### 步骤1: 在Jetson上打包

```bash
# 创建最小化包
mkdir -p ~/isspa_pc_minimal/src/isspa_object_detection/camera_based_detection
mkdir -p ~/isspa_pc_minimal/src/isspa_navigation/navigation_stack/launch

# 复制yolov5_ros包
cp -r ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_object_detection/camera_based_detection/yolov5_ros \
    ~/isspa_pc_minimal/src/isspa_object_detection/camera_based_detection/

# 复制launch文件
cp ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_navigation/navigation_stack/launch/pavs_navigation_pc_only.launch \
    ~/isspa_pc_minimal/src/isspa_navigation/navigation_stack/launch/

# 打包
cd ~/isspa_pc_minimal
tar -czf ~/isspa_pc_minimal.tar.gz src/
```

### 步骤2: 传输到PC

```bash
# 使用scp
scp ~/isspa_pc_minimal.tar.gz flyx@<PC_IP>:~/Desktop/

# 或使用U盘等其他方式
```

### 步骤3: 在PC上安装

```bash
# 1. 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# 2. 解压
tar -xzf ~/Desktop/isspa_pc_minimal.tar.gz

# 3. 创建package.xml（如果缺失）
cat > src/isspa_navigation/navigation_stack/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>navigation_stack</name>
  <version>1.0.0</version>
  <description>Navigation stack</description>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
</package>
EOF

# 4. 创建CMakeLists.txt（如果缺失）
cat > src/isspa_navigation/navigation_stack/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(navigation_stack)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
EOF

# 5. 安装依赖
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

pip3 install ultralytics opencv-python numpy

# 6. 编译
catkin_make
source devel/setup.bash

# 7. 测试
rospack find yolov5_ros
```

## 方法3: 完整克隆（如果要在PC上开发）

```bash
# 在PC上
cd ~/catkin_ws/src
git clone <your-repo-url> ISSPA

# 或从Jetson同步
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
```

## 常见问题

### 问题1: 找不到包

```bash
# 确保已source
source ~/catkin_ws/devel/setup.bash

# 检查ROS_PACKAGE_PATH
echo $ROS_PACKAGE_PATH
```

### 问题2: 编译失败

```bash
# 检查缺失的package.xml或CMakeLists.txt
# 参考上面的手动安装步骤创建它们
```

### 问题3: Python模块缺失

```bash
# 安装依赖
pip3 install -r $(rospack find yolov5_ros)/scripts/requirements.txt
```

