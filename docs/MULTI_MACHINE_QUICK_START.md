# 多机ROS通信快速开始
# Multi-Machine ROS Quick Start

## 快速配置（5分钟）

### 步骤1: 在PC上配置

```bash
# 1. 运行配置脚本
chmod +x scripts/setup_multi_machine.sh
./scripts/setup_multi_machine.sh
# 选择: 1 (PC)

# 2. 启动roscore
roscore

# 3. 在另一个终端启动检测节点
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    model_size:=s \
    image_size:=640
```

### 步骤2: 在Jetson上配置

```bash
# 1. 运行配置脚本
chmod +x scripts/setup_multi_machine.sh
./scripts/setup_multi_machine.sh
# 选择: 2 (Jetson)
# 输入PC的IP地址

# 2. 测试连接（确保PC端roscore已启动）
rostopic list

# 3. 启动小车节点
roslaunch navigation_stack pavs_navigation_vehicle_only.launch
```

## 手动配置（如果脚本不工作）

### PC端

```bash
# 查看IP地址
hostname -I

# 假设IP是 192.168.1.100
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.100

# 添加到 ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.1.100:11311" >> ~/.bashrc
echo "export ROS_IP=192.168.1.100" >> ~/.bashrc

# 启动roscore
roscore
```

### Jetson端

```bash
# 查看IP地址
hostname -I

# 假设Jetson IP是 172.21.165.177，PC IP是 192.168.1.100
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=172.21.165.177

# 添加到 ~/.bashrc
echo "export ROS_MASTER_URI=http://192.168.1.100:11311" >> ~/.bashrc
echo "export ROS_IP=172.21.165.177" >> ~/.bashrc

# 测试连接
rostopic list
```

## 性能对比

| 配置 | FPS | 显存占用 | 延迟 |
|------|-----|----------|------|
| Jetson (yolov5n, 416) | 15-20 | 1.5GB | 0ms |
| PC (yolov5s, 640) | 30-60 | 2-3GB | 50-100ms |
| PC (yolov5m, 640) | 20-40 | 4-5GB | 50-100ms |

## 优势

1. **速度**: PC GPU通常比Jetson快2-3倍
2. **模型**: 可以使用更大的模型（yolov5s/m）提高精度
3. **负担**: Jetson只负责传感器，计算负担小
4. **调试**: 在PC上更容易调试和可视化

## 注意事项

1. **网络**: 确保PC和Jetson在同一网络
2. **延迟**: 网络延迟通常<100ms，对红绿灯检测影响不大
3. **带宽**: 640x480图像约1-2MB/s，确保网络带宽足够
4. **顺序**: 先启动PC端roscore，再启动Jetson端节点

## 故障排查

### 无法连接

```bash
# 检查网络
ping <PC_IP>

# 检查ROS Master
rostopic list

# 检查防火墙
sudo ufw status
```

### 话题无数据

```bash
# 检查话题
rostopic list

# 检查话题数据
rostopic echo /camera/color/image_raw --noarr
```

### 延迟太大

```bash
# 降低图像分辨率
# 在launch文件中设置较小的分辨率，如480x360
```

