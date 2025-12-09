# 多机ROS通讯完整指南
# Multi-Machine ROS Communication Complete Guide

## 📋 目录

1. [快速开始](#快速开始)
2. [详细配置步骤](#详细配置步骤)
3. [启动方式](#启动方式)
4. [性能优化](#性能优化)
5. [故障排查](#故障排查)

---

## 🚀 快速开始（5分钟）

### 步骤1: PC端配置

```bash
# 1. 查看PC的IP地址
hostname -I  # Linux
ipconfig     # Windows

# 2. 设置ROS环境变量（假设PC IP是 192.168.1.100）
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100

# 3. 添加到 ~/.bashrc（永久生效）
echo "export ROS_MASTER_URI=http://192.168.1.100:11311" >> ~/.bashrc
echo "export ROS_IP=192.168.1.100" >> ~/.bashrc

# 4. 启动roscore（第一个终端）
roscore

# 5. 启动检测节点（第二个终端）
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_cpu:=true \
    model_size:=n \
    image_size:=416
```

### 步骤2: Jetson端配置

```bash
# 1. 查看Jetson的IP地址
hostname -I

# 2. 设置ROS环境变量（假设Jetson IP是 172.21.165.177，PC IP是 192.168.1.100）
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=172.21.165.177

# 3. 添加到 ~/.bashrc（永久生效）
echo "export ROS_MASTER_URI=http://192.168.1.100:11311" >> ~/.bashrc
echo "export ROS_IP=172.21.165.177" >> ~/.bashrc

# 4. 测试连接（确保PC端roscore已启动）
rostopic list

# 5. 启动小车节点
roslaunch navigation_stack pavs_navigation_vehicle_only.launch
```

---

## 📝 详细配置步骤

### 一、PC端安装（首次使用需要）

#### 方法1: 使用脚本（推荐）

```bash
# 在Jetson上创建安装包
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light
chmod +x scripts/create_pc_minimal_package.sh
./scripts/create_pc_minimal_package.sh

# 传输到PC（使用scp、U盘等）
scp ~/isspa_pc_minimal.tar.gz flyx@<PC_IP>:~/Desktop/

# 在PC上安装
chmod +x scripts/install_pc_minimal.sh
./scripts/install_pc_minimal.sh
source ~/catkin_ws/devel/setup.bash
```

#### 方法2: 手动安装

```bash
# 1. 在Jetson上打包
mkdir -p ~/isspa_pc_minimal/src/isspa_object_detection/camera_based_detection
mkdir -p ~/isspa_pc_minimal/src/isspa_navigation/navigation_stack/launch

cp -r ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_object_detection/camera_based_detection/yolov5_ros \
    ~/isspa_pc_minimal/src/isspa_object_detection/camera_based_detection/

cp ~/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_navigation/navigation_stack/launch/pavs_navigation_pc_only.launch \
    ~/isspa_pc_minimal/src/isspa_navigation/navigation_stack/launch/

cd ~/isspa_pc_minimal
tar -czf ~/isspa_pc_minimal.tar.gz src/

# 2. 在PC上解压和编译
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
tar -xzf ~/Desktop/isspa_pc_minimal.tar.gz

# 创建package.xml
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

# 创建CMakeLists.txt
cat > src/isspa_navigation/navigation_stack/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(navigation_stack)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
EOF

# 安装依赖
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

# 编译
catkin_make
source devel/setup.bash
```

### 二、网络配置

#### PC端

```bash
# 1. 查看IP地址
hostname -I  # Linux
ipconfig     # Windows

# 2. 设置ROS环境变量（替换为你的PC IP）
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<PC_IP>

# 3. 配置防火墙（开放ROS端口）
sudo ufw allow 11311/tcp
sudo ufw allow 11312/tcp

# 4. 永久设置
echo "export ROS_MASTER_URI=http://<PC_IP>:11311" >> ~/.bashrc
echo "export ROS_IP=<PC_IP>" >> ~/.bashrc
```

#### Jetson端

```bash
# 1. 查看IP地址
hostname -I

# 2. 设置ROS环境变量（替换为你的IP）
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<JETSON_IP>

# 3. 永久设置
echo "export ROS_MASTER_URI=http://<PC_IP>:11311" >> ~/.bashrc
echo "export ROS_IP=<JETSON_IP>" >> ~/.bashrc
```

### 三、测试连接

```bash
# 在PC上启动roscore
roscore

# 在Jetson上测试
rostopic list  # 应该能看到PC上的话题
ping <PC_IP>   # 测试网络连接
```

---

## 🎯 启动方式

### PC端启动（运行YOLOv5检测）

#### CPU模式（虚拟机推荐）

```bash
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_cpu:=true \
    model_size:=n \
    image_size:=416
```

#### GPU模式（如果有GPU）

```bash
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_pc_gpu:=true \
    model_size:=s \
    image_size:=640
```

### Jetson端启动（运行底盘和传感器）

```bash
roslaunch navigation_stack pavs_navigation_vehicle_only.launch
```

### 启动顺序

1. **先启动PC端**：roscore → 检测节点
2. **再启动Jetson端**：小车节点

---

## ⚡ 性能优化

### CPU模式优化（虚拟机）

```bash
# 使用最小配置
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_cpu:=true \
    model_size:=n \
    image_size:=320
```

### GPU模式优化（有GPU的PC）

```bash
# 使用更大模型和分辨率
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_pc_gpu:=true \
    model_size:=s \
    image_size:=640
```

### 性能对比

| 配置 | FPS | 延迟 | 适用场景 |
|------|-----|------|----------|
| CPU (yolov5n, 320) | 5-12 | 80-200ms | 虚拟机/无GPU |
| CPU (yolov5n, 416) | 3-8 | 150-300ms | 虚拟机/无GPU |
| GPU (yolov5s, 640) | 30-60 | 20-30ms | 有GPU的PC |

---

## 🔧 故障排查

### 问题1: 无法连接到ROS Master

**症状**: `rostopic list` 失败

**解决**:
```bash
# 检查网络连接
ping <PC_IP>

# 检查ROS_MASTER_URI
echo $ROS_MASTER_URI

# 检查防火墙
sudo ufw status

# 确保PC端roscore已启动
```

### 问题2: 话题无数据

**症状**: 话题存在但没有数据

**解决**:
```bash
# 检查话题
rostopic list

# 检查话题数据
rostopic echo /camera/color/image_raw --noarr

# 检查话题频率
rostopic hz /camera/color/image_raw
```

### 问题3: 节点启动失败

**症状**: 节点立即退出

**解决**:
```bash
# 查看日志
cat ~/.ros/log/latest/*/yolov5_traffic_light_detection-*.log

# 检查包是否存在
rospack find yolov5_ros

# 检查Python依赖
python3 -c "import ultralytics; print('OK')"
```

### 问题4: 图像传输延迟大

**解决**:
```bash
# 降低图像分辨率（在launch文件中）
image_size:=320

# 或使用图像压缩
rosrun image_transport republish raw compressed
```

### 问题5: CPU推理太慢

**解决**:
```bash
# 使用最小配置
model_size:=n
image_size:=320

# 降低检测频率（修改detect_ros.py中的rate）
rate = rospy.Rate(5)  # 5 Hz instead of 10 Hz
```

---

## 📊 架构说明

```
┌─────────────────┐         ┌──────────────────┐
│    Jetson小车   │  <--->  │    PC (主机)     │
│                 │         │                  │
│ - 底盘节点      │         │ - YOLOv5检测     │
│ - 摄像头节点    │         │ - 决策节点       │
│ - 导航节点      │         │ - 可视化节点     │
│ - 传感器节点    │         │                  │
└─────────────────┘         └──────────────────┘
```

**数据流**:
- Jetson → PC: `/camera/color/image_raw` (图像)
- PC → Jetson: `/cmd_vel` (控制命令)

---

## ✅ 验证清单

启动前检查：

- [ ] PC和Jetson在同一网络
- [ ] PC端已安装项目代码并编译
- [ ] PC端已设置ROS_MASTER_URI和ROS_IP
- [ ] Jetson端已设置ROS_MASTER_URI和ROS_IP
- [ ] PC端防火墙已开放11311端口
- [ ] PC端roscore已启动
- [ ] 可以ping通对方IP
- [ ] rostopic list可以正常工作

---

## 📞 快速参考

### PC端命令

```bash
# 启动roscore
roscore

# 启动检测节点（CPU模式）
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_cpu:=true \
    model_size:=n \
    image_size:=416

# 检查节点
rosnode list | grep yolov5
```

### Jetson端命令

```bash
# 测试连接
rostopic list

# 启动小车节点
roslaunch navigation_stack pavs_navigation_vehicle_only.launch

# 检查话题
rostopic echo /camera/color/image_raw --noarr
```

---

## 📚 相关文件

- `pavs_navigation_pc_only.launch` - PC端启动文件
- `pavs_navigation_vehicle_only.launch` - Jetson端启动文件
- `scripts/create_pc_minimal_package.sh` - 创建PC安装包脚本
- `scripts/install_pc_minimal.sh` - PC端安装脚本

---

**最后更新**: 2024年

