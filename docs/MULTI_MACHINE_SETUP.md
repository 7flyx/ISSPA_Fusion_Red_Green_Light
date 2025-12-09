# 多机ROS通信配置指南
# Multi-Machine ROS Communication Setup Guide

## 架构说明 / Architecture

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

## 配置步骤 / Setup Steps

### 步骤1: 配置网络

#### 在Jetson上（小车端）

```bash
# 查看IP地址
hostname -I

# 假设Jetson IP: 172.21.165.177
# 设置ROS_MASTER_URI指向PC
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=172.21.165.177

# 永久设置（添加到 ~/.bashrc）
echo "export ROS_MASTER_URI=http://<PC_IP>:11311" >> ~/.bashrc
echo "export ROS_IP=172.21.165.177" >> ~/.bashrc
```

#### 在PC上（主机端）

```bash
# 查看IP地址
hostname -I  # Linux
ipconfig     # Windows

# 假设PC IP: 192.168.1.100
# 设置ROS_MASTER_URI指向自己
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100

# 永久设置（添加到 ~/.bashrc 或 ~/.zshrc）
echo "export ROS_MASTER_URI=http://192.168.1.100:11311" >> ~/.bashrc
echo "export ROS_IP=192.168.1.100" >> ~/.bashrc
```

### 步骤2: 配置防火墙

#### 在PC上（需要开放ROS端口）

```bash
# Ubuntu/Debian
sudo ufw allow 11311/tcp
sudo ufw allow 11312/tcp

# 或者关闭防火墙（仅用于测试）
sudo ufw disable
```

#### Windows防火墙

1. 打开"Windows Defender 防火墙"
2. 点击"高级设置"
3. 添加入站规则：
   - 端口：11311 (TCP)
   - 端口：11312 (TCP)

### 步骤3: 配置hosts文件（可选，但推荐）

#### 在Jetson上

```bash
sudo nano /etc/hosts
# 添加：
192.168.1.100    pc-hostname
```

#### 在PC上

```bash
sudo nano /etc/hosts  # Linux
# 或编辑 C:\Windows\System32\drivers\etc\hosts  # Windows
# 添加：
172.21.165.177    jetson-desktop
```

### 步骤4: 测试连接

#### 在PC上启动roscore

```bash
# 在PC上
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100
roscore
```

#### 在Jetson上测试

```bash
# 在Jetson上
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=172.21.165.177
rostopic list  # 应该能看到PC上的话题
```

## 启动方式 / Launch Methods

### 方式1: 小车端只启动基础节点

在Jetson上运行：

```bash
roslaunch navigation_stack pavs_navigation_vehicle_only.launch
```

这会启动：
- 底盘节点
- 摄像头节点
- 导航节点
- 传感器节点
- **不启动** YOLOv5检测节点

### 方式2: PC端启动检测和决策节点

在PC上运行：

```bash
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true
```

这会启动：
- YOLOv5检测节点（使用PC的GPU）
- 红绿灯决策节点
- 可视化节点
- **不启动** 底盘和导航节点

## 性能优势 / Performance Benefits

1. **计算资源**: PC通常有更强的GPU（如RTX系列），推理速度更快
2. **显存**: PC显存更大，可以使用更大的模型（yolov5s甚至yolov5m）
3. **Jetson负担**: 小车只负责传感器和底盘控制，计算负担小
4. **实时性**: 通过网络传输图像，延迟通常<100ms，对红绿灯检测影响不大

## 注意事项 / Notes

1. **网络延迟**: 确保网络稳定，延迟<100ms
2. **带宽**: 图像传输需要一定带宽（640x480约1-2MB/s）
3. **同步**: 确保PC和Jetson时间同步（使用NTP）
4. **话题名称**: 确保话题名称匹配

## 故障排查 / Troubleshooting

### 问题1: 无法连接到ROS Master

```bash
# 检查网络连接
ping <PC_IP>

# 检查ROS Master是否运行
rostopic list

# 检查防火墙
sudo ufw status
```

### 问题2: 话题无法接收

```bash
# 检查话题是否存在
rostopic list

# 检查话题数据
rostopic echo /camera/color/image_raw --noarr
```

### 问题3: 图像传输延迟大

```bash
# 降低图像分辨率
# 在launch文件中设置较小的分辨率

# 或使用图像压缩
rosrun image_transport republish raw compressed
```

