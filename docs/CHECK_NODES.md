# 检查节点运行状态
# Check Node Running Status

## 快速检查命令

在启动launch文件后，运行以下命令检查节点状态：

```bash
# 1. 查看所有节点
rosnode list

# 2. 检查关键节点是否存在
rosnode list | grep -E "(camera|yolov5|traffic_light|visualizer)"

# 3. 检查话题
rostopic list | grep -E "(camera|detection|traffic_light)"

# 4. 查看节点详细信息（如果节点存在）
rosnode info /yolov5_traffic_light_detection
rosnode info /traffic_light_visualizer
rosnode info /smart_traffic_light_decision
```

## 如果节点没有启动

### 检查日志文件

```bash
# 查看最新的日志目录
ls -lt ~/.ros/log/ | head -5

# 查看特定节点的日志
cat ~/.ros/log/latest/*/yolov5_traffic_light_detection-*.log
cat ~/.ros/log/latest/*/smart_traffic_light_decision-*.log
cat ~/.ros/log/latest/*/traffic_light_visualizer-*.log
```

### 手动测试节点

```bash
# 测试红绿灯检测节点
rosrun yolov5_ros detect_ros.py --weights /path/to/model.pt --source /camera/color/image_raw

# 测试可视化节点
rosrun yolov5_ros traffic_light_visualizer.py

# 测试决策节点
rosrun yolov5_ros traffic_light_decision_smart_turn.py
```

## 常见错误

### 错误1: 找不到包

```bash
# 检查包路径
rospack find yolov5_ros

# 如果找不到，重新source
source ~/catkin_ws/devel/setup.bash
```

### 错误2: 找不到模型文件

```bash
# 检查模型文件是否存在
ls -l $(rospack find yolov5_ros)/scripts/yolov5*.pt

# 如果不存在，需要先训练模型或下载
```

### 错误3: 摄像头话题不存在

```bash
# 检查摄像头话题
rostopic list | grep camera

# 如果没有，检查摄像头是否启动
rosnode list | grep camera
```

