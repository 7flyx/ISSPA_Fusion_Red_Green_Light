# 手动启动红绿灯检测节点
# Manually Start Traffic Light Detection Nodes

## 如果节点没有自动启动

### 步骤1: 检查摄像头话题

```bash
# 查看摄像头话题是否有数据
rostopic echo /camera/color/image_raw --noarr -n 1

# 如果话题名称不同，查找正确的摄像头话题
rostopic list | grep image
```

### 步骤2: 手动启动红绿灯检测节点

```bash
# 启动检测节点
rosrun yolov5_ros detect_ros.py \
    --weights $(rospack find yolov5_ros)/scripts/yolov5n_traffic_lights.pt \
    --source /camera/color/image_raw \
    --img 416 \
    --half \
    --conf-thres 0.5
```

### 步骤3: 手动启动决策节点

在另一个终端：

```bash
rosrun yolov5_ros traffic_light_decision_smart_turn.py
```

### 步骤4: 手动启动可视化节点

在另一个终端：

```bash
rosrun yolov5_ros traffic_light_visualizer.py
```

## 检查节点是否正常运行

```bash
# 检查节点
rosnode list | grep -E "(yolov5|traffic_light|visualizer)"

# 检查话题
rostopic list | grep -E "(detection|traffic_light)"

# 查看检测结果
rostopic echo /detection_results --noarr
rostopic echo /traffic_light_state
```

## 常见错误

### 错误1: 找不到模型文件

```bash
# 检查模型文件
ls -l $(rospack find yolov5_ros)/scripts/yolov5*.pt

# 如果不存在，需要先训练或下载模型
```

### 错误2: 找不到摄像头话题

```bash
# 查找正确的摄像头话题
rostopic list | grep image

# 如果话题不同，修改source参数
# 例如：--source /usb_cam/image_raw
```

### 错误3: ultralytics未安装

```bash
# 安装ultralytics
pip3 install "ultralytics<8.3.0"
```

