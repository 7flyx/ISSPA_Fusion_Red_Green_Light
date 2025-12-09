# 节点调试指南
# Node Debugging Guide

## 检查节点状态

### 查看所有运行的节点

```bash
rosnode list
```

### 查看特定节点信息

```bash
# 检查摄像头节点
rosnode info /camera/camera

# 检查红绿灯检测节点
rosnode info /yolov5_traffic_light_detection

# 检查可视化节点
rosnode info /traffic_light_visualizer

# 检查决策节点
rosnode info /smart_traffic_light_decision
```

### 查看节点日志

```bash
# 查看最近的日志
tail -f ~/.ros/log/latest/*/yolov5_traffic_light_detection-*.log
tail -f ~/.ros/log/latest/*/smart_traffic_light_decision-*.log
tail -f ~/.ros/log/latest/*/traffic_light_visualizer-*.log
```

## 检查话题

### 查看所有话题

```bash
rostopic list
```

### 检查关键话题是否有数据

```bash
# 检查摄像头图像
rostopic echo /camera/color/image_raw --noarr

# 检查检测结果
rostopic echo /detection_results --noarr

# 检查红绿灯状态
rostopic echo /traffic_light_state

# 检查决策结果
rostopic echo /traffic_light_decision
```

## 手动启动节点测试

### 测试摄像头

```bash
# 方法1: 使用Astra
roslaunch astra_camera astra.launch

# 方法2: 使用USB摄像头
rosrun usb_cam usb_cam_node _video_device:=/dev/video0
```

### 测试红绿灯检测

```bash
rosrun yolov5_ros detect_ros.py \
    --weights /path/to/model.pt \
    --source /camera/color/image_raw \
    --img 416 \
    --half
```

### 测试可视化

```bash
rosrun yolov5_ros traffic_light_visualizer.py
```

## 常见问题

### 问题1: 节点启动后立即退出

**检查**:
```bash
# 查看节点退出代码
rosnode list | grep -v "/rosout"

# 查看日志文件
ls -lt ~/.ros/log/latest/*/
```

### 问题2: 找不到包

```bash
# 检查包是否存在
rospack find yolov5_ros
rospack find astra_camera

# 如果找不到，重新source
source ~/catkin_ws/devel/setup.bash
```

### 问题3: 权限问题

```bash
# 检查脚本是否有执行权限
ls -l ~/catkin_ws/devel/lib/yolov5_ros/*.py

# 如果没有，添加权限
chmod +x ~/catkin_ws/devel/lib/yolov5_ros/*.py
```

