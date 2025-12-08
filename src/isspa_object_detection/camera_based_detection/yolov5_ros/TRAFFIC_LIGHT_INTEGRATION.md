# 红绿灯检测与决策集成指南
# Traffic Light Detection and Decision Integration Guide

## 概述 / Overview

本文档说明如何将红绿灯检测模型集成到ISSPA自动驾驶项目中，并实现基于红绿灯状态的决策控制。

This document explains how to integrate a traffic light detection model into the ISSPA autonomous driving project and implement decision control based on traffic light states.

## 系统架构 / System Architecture

```
摄像头图像 (Camera Image)
    ↓
[YOLOv5检测模块] → 红绿灯状态 (/traffic_light_state)
    ↓
[决策模块] → 决策结果 (/traffic_light_decision) + 速度命令 (/cmd_vel)
    ↓
[车辆底盘] → 执行控制
```

## 组件说明 / Components

### 1. 红绿灯检测模块 (Traffic Light Detection Module)

**文件**: `scripts/detect_ros.py`

**功能**:
- 使用YOLOv5模型检测图像中的红绿灯
- 识别红绿灯状态（红/绿/黄）
- 发布检测结果到 `/traffic_light_state` 话题

**消息类型**: `yolov5_ros/TrafficLightState`
- `state`: 0=未检测到, 1=红灯, 2=绿灯, 3=黄灯
- `confidence`: 检测置信度
- `frame_id`: 图像帧ID
- `stamp`: 时间戳

### 2. 红绿灯决策模块 (Traffic Light Decision Module)

**文件**: `scripts/traffic_light_decision.py`

**功能**:
- 接收红绿灯检测结果
- 根据红绿灯状态做出决策：
  - **红灯**: 停止车辆（发布零速度命令）
  - **绿灯**: 允许通行（转发导航栈的速度命令）
  - **黄灯**: 根据配置停止或减速
  - **未检测到**: 保持当前状态（转发导航栈的速度命令）

**订阅话题**:
- `/traffic_light_state`: 红绿灯检测结果
- `/cmd_vel_nav`: 导航栈的原始速度命令

**发布话题**:
- `/cmd_vel`: 融合了红绿灯决策的最终速度命令
- `/traffic_light_decision`: 决策结果（供其他模块使用）
- `/traffic_light_should_stop`: 是否应该停止的标志

## 安装与配置 / Installation and Configuration

### 1. 编译ROS包

```bash
cd ~/catkin_ws  # 或你的工作空间目录
catkin_make
source devel/setup.bash
```

### 2. 配置模型路径

确保你的红绿灯检测模型文件位于正确的位置：
- 默认路径: `src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts/yolov5s_traffic_lights.pt`
- 或在launch文件中指定模型路径

### 3. 配置类别名称

在 `detect_ros.py` 中，根据你的模型训练时的类别名称，调整 `traffic_light_classes` 字典：

```python
traffic_light_classes = {
    'red': 1,
    'green': 2,
    'yellow': 3,
    'traffic_light_red': 1,    # 如果模型使用这种命名
    'traffic_light_green': 2,
    'traffic_light_yellow': 3,
}
```

## 使用方法 / Usage

### 方法1: 使用Launch文件（推荐）

```bash
roslaunch yolov5_ros traffic_light_integration.launch \
    camera_topic:=/camera/color/image_raw \
    model_path:=/path/to/your/model.pt \
    confidence_threshold:=0.5 \
    yellow_light_action:=stop
```

### 方法2: 单独启动节点

```bash
# 终端1: 启动检测节点
rosrun yolov5_ros detect_ros.py \
    --weights /path/to/your/model.pt \
    --source /camera/color/image_raw

# 终端2: 启动决策节点
rosrun yolov5_ros traffic_light_decision.py
```

### 方法3: 与导航系统集成

为了与导航系统正确集成，需要修改导航栈的配置，使其将速度命令发布到 `/cmd_vel_nav` 而不是直接发布到 `/cmd_vel`。

**修改导航栈配置**:

在导航栈的配置文件中（如 `move_base_params.yaml`），可以创建一个速度命令重映射：

```xml
<!-- 在导航栈的launch文件中 -->
<node name="move_base" ...>
    <remap from="/cmd_vel" to="/cmd_vel_nav"/>
    ...
</node>
```

这样，导航栈的速度命令会发布到 `/cmd_vel_nav`，然后决策模块会将其与红绿灯决策融合后发布到 `/cmd_vel`。

## 参数配置 / Parameter Configuration

### 检测模块参数

- `confidence_threshold`: 检测置信度阈值（默认: 0.5）
- `weights`: 模型文件路径
- `source`: 图像源（摄像头话题或设备ID）

### 决策模块参数

- `confidence_threshold`: 决策使用的置信度阈值（默认: 0.5）
- `yellow_light_action`: 黄灯时的行为（'stop' 或 'slow'，默认: 'stop'）
- `slow_down_speed`: 减速时的最大速度（默认: 0.2 m/s）

## 测试与调试 / Testing and Debugging

### 1. 查看检测结果

```bash
# 查看红绿灯状态话题
rostopic echo /traffic_light_state

# 查看决策结果
rostopic echo /traffic_light_decision

# 查看是否应该停止
rostopic echo /traffic_light_should_stop
```

### 2. 可视化检测结果

如果检测节点配置了 `view_img:=true`，会显示检测结果窗口。

### 3. 调试日志

决策模块会输出日志信息：
- `RED LIGHT: Vehicle stopped` - 检测到红灯
- `GREEN LIGHT: Vehicle allowed to proceed` - 检测到绿灯
- `YELLOW LIGHT: Vehicle stopped/slowing down` - 检测到黄灯

## 训练自己的模型 / Training Your Own Model

如果你需要训练自己的红绿灯检测模型：

1. **准备数据集**: 收集包含红绿灯的图像，标注类别（红/绿/黄）

2. **训练YOLOv5模型**: 
   ```bash
   cd src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts
   python train.py --data your_dataset.yaml --weights yolov5s.pt --epochs 100
   ```

3. **更新类别名称**: 确保 `detect_ros.py` 中的类别名称与训练时一致

4. **测试模型**: 使用训练好的模型进行检测测试

## 常见问题 / FAQ

### Q1: 检测不到红绿灯怎么办？

- 检查模型路径是否正确
- 检查摄像头话题是否正确
- 降低置信度阈值
- 检查类别名称是否匹配

### Q2: 车辆在绿灯时也不动？

- 检查 `/cmd_vel_nav` 话题是否有数据
- 检查决策模块是否正常运行
- 查看决策模块的日志输出

### Q3: 如何调整决策行为？

修改 `traffic_light_decision.py` 中的决策逻辑，或通过ROS参数动态配置。

## 扩展功能 / Extended Features

### 1. 添加距离估计

可以在检测结果中添加红绿灯的距离信息，用于更精确的决策。

### 2. 多红绿灯处理

如果场景中有多个红绿灯，可以扩展决策模块来处理多个检测结果。

### 3. 与高精地图集成

结合高精地图中的红绿灯位置信息，提高检测和决策的准确性。

## 贡献 / Contributing

欢迎提交问题和改进建议！

## 许可证 / License

与ISSPA项目保持一致。



