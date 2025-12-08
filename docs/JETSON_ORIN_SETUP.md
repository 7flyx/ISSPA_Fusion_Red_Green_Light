# Jetson Orin Nano Super 4GB 环境配置指南
# Jetson Orin Nano Super 4GB Environment Setup Guide

## 系统要求 / System Requirements

- **硬件**: NVIDIA Jetson Orin Nano Super 4GB
- **系统**: JetPack 5.x (Ubuntu 20.04/22.04)
- **ROS**: ROS Noetic (Ubuntu 20.04) 或 ROS 2 Humble (Ubuntu 22.04)

## 1. 系统准备 / System Preparation

### 1.1 检查系统版本

```bash
# 检查JetPack版本
cat /etc/nv_tegra_release

# 检查CUDA版本
nvcc --version

# 检查Python版本
python3 --version
```

### 1.2 更新系统

```bash
sudo apt-get update
sudo apt-get upgrade -y
```

### 1.3 安装基础工具

```bash
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    htop
```

## 2. ROS安装 / ROS Installation

### 2.1 安装ROS Noetic (Ubuntu 20.04)

```bash
# 设置源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 安装ROS
sudo apt-get update
sudo apt-get install -y ros-noetic-desktop-full

# 初始化rosdep
sudo rosdep init
rosdep update

# 设置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安装依赖
sudo apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool
```

### 2.2 安装ROS 2 Humble (Ubuntu 22.04)

```bash
# 设置源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# 安装ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop

# 设置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 3. CUDA和深度学习环境 / CUDA and Deep Learning Environment

### 3.1 检查CUDA（通常JetPack已包含）

```bash
# 检查CUDA版本
nvcc --version

# 检查GPU
nvidia-smi
```

### 3.2 安装PyTorch for Jetson

Jetson平台需要安装专门为ARM架构编译的PyTorch：

```bash
# 下载PyTorch for Jetson
# 访问: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

# 示例：安装PyTorch 1.13.0 (根据JetPack版本选择)
wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.13.0-cp38-cp38-linux_aarch64.whl

# 安装依赖
sudo apt-get install -y python3-pip libopenblas-base libopenmpi-dev libomp-dev
pip3 install Cython numpy

# 安装PyTorch
pip3 install torch-1.13.0-cp38-cp38-linux_aarch64.whl

# 验证安装
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
```

### 3.3 安装TorchVision

```bash
# 安装依赖
sudo apt-get install -y libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev

# 安装TorchVision
git clone --branch v0.14.0 https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.14.0
python3 setup.py install
cd ..
```

### 3.4 安装其他深度学习依赖

```bash
pip3 install \
    ultralytics \
    opencv-python \
    pillow \
    matplotlib \
    scipy \
    tqdm \
    pyyaml \
    requests
```

## 4. YOLOv5优化配置 / YOLOv5 Optimization

### 4.1 使用TensorRT加速（推荐）

```bash
# 安装TensorRT（通常JetPack已包含）
# 检查TensorRT版本
dpkg -l | grep tensorrt

# 安装TensorRT Python API
pip3 install nvidia-tensorrt
```

### 4.2 优化YOLOv5配置

在 `detect_ros.py` 中，针对Jetson优化：

```python
# 使用FP16精度（减少显存占用，提高速度）
half=True

# 使用较小的模型（yolov5n或yolov5s）
weights='yolov5n.pt'  # 或 yolov5s.pt

# 降低输入分辨率
imgsz=416  # 而不是640
```

### 4.3 创建Jetson优化配置

创建 `jetson_config.yaml`:

```yaml
# Jetson优化配置
device: 0  # 使用GPU
half: true  # FP16精度
imgsz: 416  # 输入尺寸
batch_size: 1  # 批次大小
workers: 2  # 数据加载线程数
```

## 5. ROS工作空间配置 / ROS Workspace Setup

### 5.1 创建工作空间

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆ISSPA项目（如果还没有）
# git clone <your-isspa-repo>

cd ~/catkin_ws
```

### 5.2 安装ROS依赖

```bash
# 安装ISSPA所需的ROS包
sudo apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-image-transport-plugins \
    ros-noetic-camera-info-manager \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-topic-tools
```

### 5.3 编译工作空间

```bash
cd ~/catkin_ws
catkin_make

# 如果遇到内存不足，使用单线程编译
catkin_make -j1

# 设置环境变量
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. 摄像头配置 / Camera Configuration

### 6.1 检查摄像头设备

```bash
# 列出所有视频设备
ls -l /dev/video*

# 测试摄像头
v4l2-ctl --list-devices
```

### 6.2 安装摄像头驱动

#### Astra摄像头

```bash
# 如果使用Astra摄像头，安装驱动
cd ~/catkin_ws/src
git clone https://github.com/orbbec/ros_astra_camera.git
cd ~/catkin_ws
catkin_make
```

#### USB摄像头

```bash
# 安装usb_cam
sudo apt-get install -y ros-noetic-usb-cam

# 或从源码编译
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/usb_cam.git
cd ~/catkin_ws
catkin_make
```

### 6.3 测试摄像头

```bash
# 使用usb_cam测试
rosrun usb_cam usb_cam_node _video_device:=/dev/video0

# 查看图像
rosrun image_view image_view image:=/usb_cam/image_raw
```

## 7. 性能优化 / Performance Optimization

### 7.1 Jetson性能模式

```bash
# 设置最大性能模式
sudo nvpmodel -m 0
sudo jetson_clocks

# 检查当前模式
sudo nvpmodel -q
```

### 7.2 显存管理

Jetson Orin Nano Super 4GB只有4GB显存，需要优化：

```python
# 在detect_ros.py中
import torch

# 清理显存缓存
torch.cuda.empty_cache()

# 使用混合精度
from torch.cuda.amp import autocast
```

### 7.3 降低模型复杂度

```bash
# 使用最小的模型
# yolov5n.pt 而不是 yolov5s.pt

# 降低输入分辨率
# 416x416 而不是 640x640
```

### 7.4 优化ROS节点

```python
# 降低发布频率
rate = rospy.Rate(5)  # 5 Hz instead of 10 Hz

# 减少图像分辨率
scaling_factor = 0.5  # 缩小图像
```

## 8. 系统监控 / System Monitoring

### 8.1 监控GPU使用

```bash
# 实时监控
watch -n 1 nvidia-smi

# 或使用jtop（推荐）
sudo pip3 install -U jetson-stats
sudo systemctl restart jetson_stats.service
jtop
```

### 8.2 监控系统资源

```bash
# CPU和内存
htop

# 温度
cat /sys/devices/virtual/thermal/thermal_zone*/temp
```

## 9. 常见问题解决 / Troubleshooting

### 问题1: 显存不足 (Out of Memory)

**解决**:
```bash
# 1. 使用更小的模型
weights='yolov5n.pt'

# 2. 降低输入分辨率
imgsz=416

# 3. 使用FP16
half=True

# 4. 减少batch size
batch_size=1
```

### 问题2: 编译时内存不足

**解决**:
```bash
# 使用单线程编译
catkin_make -j1

# 或增加swap空间
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

### 问题3: PyTorch无法使用CUDA

**解决**:
```bash
# 检查CUDA可用性
python3 -c "import torch; print(torch.cuda.is_available())"

# 如果返回False，重新安装PyTorch for Jetson
# 确保安装的是ARM版本的PyTorch
```

### 问题4: 摄像头无法识别

**解决**:
```bash
# 检查设备权限
sudo chmod 666 /dev/video0

# 检查USB连接
lsusb

# 检查驱动
dmesg | grep -i video
```

## 10. 快速启动脚本 / Quick Start Script

创建 `setup_jetson.sh`:

```bash
#!/bin/bash

echo "Setting up Jetson Orin Nano for ISSPA..."

# 设置性能模式
sudo nvpmodel -m 0
sudo jetson_clocks

# 检查环境
echo "Checking CUDA..."
nvcc --version

echo "Checking PyTorch..."
python3 -c "import torch; print('PyTorch:', torch.__version__); print('CUDA available:', torch.cuda.is_available())"

echo "Checking ROS..."
echo $ROS_DISTRO

echo "Setup complete!"
```

## 11. 验证安装 / Verification

### 11.1 测试PyTorch和CUDA

```bash
python3 << EOF
import torch
print(f"PyTorch version: {torch.__version__}")
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"CUDA version: {torch.version.cuda}")
print(f"GPU device: {torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'N/A'}")
EOF
```

### 11.2 测试YOLOv5

```bash
cd ~/catkin_ws/src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts
python3 detect.py --weights yolov5n.pt --source 0 --img 416 --half
```

### 11.3 测试ROS节点

```bash
# 启动roscore
roscore

# 在另一个终端测试节点
rosrun yolov5_ros detect_ros.py --weights yolov5n.pt --source 0
```

## 12. 推荐配置 / Recommended Configuration

### 12.1 模型选择

- **推荐**: `yolov5n.pt` (最小最快)
- **备选**: `yolov5s.pt` (平衡)

### 12.2 输入分辨率

- **推荐**: 416x416
- **备选**: 320x320 (更快但精度较低)

### 12.3 精度模式

- **推荐**: FP16 (half precision)
- **备选**: FP32 (如果精度要求高)

## 13. 性能基准 / Performance Benchmarks

在Jetson Orin Nano Super 4GB上的预期性能：

| 模型 | 分辨率 | FPS | 显存占用 |
|------|--------|-----|----------|
| yolov5n | 416x416 | ~15-20 | ~1.5GB |
| yolov5n | 320x320 | ~25-30 | ~1.2GB |
| yolov5s | 416x416 | ~8-12 | ~2.5GB |
| yolov5s | 320x320 | ~15-20 | ~2.0GB |

## 14. 下一步 / Next Steps

1. ✅ 完成系统配置
2. ✅ 测试摄像头
3. ✅ 测试YOLOv5检测
4. ✅ 启动完整系统
5. ✅ 监控性能并优化

## 相关资源 / Resources

- [NVIDIA Jetson Developer Guide](https://developer.nvidia.com/embedded/jetson-orin)
- [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)
- [YOLOv5 Documentation](https://docs.ultralytics.com/)
- [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)

