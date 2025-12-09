# 快速诊断指南
# Quick Diagnosis Guide

## 问题：红绿灯检测节点没有启动

### 立即检查

在Jetson上运行诊断脚本：

```bash
# 复制诊断脚本到Jetson
# 然后在Jetson上运行
chmod +x diagnose_traffic_light.sh
./diagnose_traffic_light.sh
```

### 手动检查步骤

#### 1. 检查节点日志

```bash
# 查看最新日志目录
LATEST_LOG=$(ls -td ~/.ros/log/* 2>/dev/null | head -1)
echo "日志目录: $LATEST_LOG"

# 查看红绿灯检测节点日志
find $LATEST_LOG -name "*yolov5_traffic_light_detection*.log" -exec cat {} \;

# 查看决策节点日志
find $LATEST_LOG -name "*smart_traffic_light_decision*.log" -exec cat {} \;
```

#### 2. 检查模型文件

```bash
# 检查模型文件是否存在
YOLOV5_PATH=$(rospack find yolov5_ros)
ls -l $YOLOV5_PATH/scripts/yolov5*.pt

# 如果不存在，需要训练或下载模型
```

#### 3. 手动测试节点

```bash
# 测试检测节点（使用正确的摄像头话题）
rosrun yolov5_ros detect_ros.py \
    --weights $(rospack find yolov5_ros)/scripts/yolov5n_traffic_lights.pt \
    --source /camera/color/image_raw \
    --img 416 \
    --half
```

#### 4. 检查ROS参数

```bash
# 查看节点参数
rosparam list | grep yolov5
rosparam get /yolov5_traffic_light_detection/weights
rosparam get /yolov5_traffic_light_detection/source
```

## 常见问题

### 问题1: 模型文件不存在

**错误**: `FileNotFoundError: yolov5n_traffic_lights.pt`

**解决**: 
```bash
# 检查模型文件
ls -l $(rospack find yolov5_ros)/scripts/yolov5*.pt

# 如果不存在，需要先训练模型
# 参考: src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts/TRAIN_TRAFFIC_LIGHT.md
```

### 问题2: 摄像头话题不存在

**错误**: `Topic /camera/color/image_raw not found`

**解决**:
```bash
# 查找正确的摄像头话题
rostopic list | grep image

# 如果话题不同，修改launch文件中的camera_topic参数
```

### 问题3: ultralytics未安装

**错误**: `ModuleNotFoundError: No module named 'ultralytics'`

**解决**:
```bash
pip3 install "ultralytics<8.3.0"
```

### 问题4: 节点立即退出

**检查日志**:
```bash
cat ~/.ros/log/latest/*/yolov5_traffic_light_detection-*.log | tail -50
```

## 重新编译和启动

修复问题后：

```bash
# 重新编译
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light
catkin_make -j1
source devel/setup.bash

# 重新启动
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    enable_camera:=true \
    enable_visualization:=true \
    camera_topic:=/camera/color/image_raw \
    use_rviz:=true \
    jetson_mode:=true \
    model_size:=n
```

