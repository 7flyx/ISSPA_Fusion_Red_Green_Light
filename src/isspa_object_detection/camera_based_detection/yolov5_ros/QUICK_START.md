# 红绿灯检测与决策快速开始指南
# Traffic Light Detection and Decision Quick Start Guide

## 前提条件 / Prerequisites

1. 已安装ROS（推荐ROS Melodic或Noetic）
2. 已编译ISSPA工作空间
3. 已训练好的红绿灯检测模型（.pt文件）

## 快速开始 / Quick Start

### 步骤1: 编译工作空间

```bash
cd ~/catkin_ws  # 或你的工作空间目录
catkin_make
source devel/setup.bash
```

### 步骤2: 准备模型文件

将你训练好的红绿灯检测模型放在：
```
src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts/yolov5s_traffic_lights.pt
```

或者修改launch文件中的模型路径。

### 步骤3: 启动红绿灯检测与决策系统

```bash
roslaunch yolov5_ros traffic_light_integration.launch \
    camera_topic:=/camera/color/image_raw \
    confidence_threshold:=0.5
```

### 步骤4: 与导航系统集成

**方法A: 修改导航栈launch文件（推荐）**

在导航栈的launch文件中，修改move_base节点：

```xml
<node pkg="move_base" type="move_base" ...>
    <!-- 将cmd_vel重映射到cmd_vel_nav -->
    <remap from="cmd_vel" to="cmd_vel_nav"/>
    ...
</node>
```

**方法B: 使用topic_tools relay**

```bash
rosrun topic_tools relay /move_base/cmd_vel /cmd_vel_nav
```

### 步骤5: 验证系统运行

```bash
# 查看红绿灯检测结果
rostopic echo /traffic_light_state

# 查看决策结果
rostopic echo /traffic_light_decision

# 查看最终速度命令
rostopic echo /cmd_vel
```

## 系统工作流程 / System Workflow

1. **检测阶段**: 摄像头图像 → YOLOv5检测 → 发布 `/traffic_light_state`
2. **决策阶段**: 接收检测结果 + 导航速度命令 → 决策模块 → 发布 `/cmd_vel`
3. **执行阶段**: 车辆底盘接收 `/cmd_vel` → 执行控制

## 决策规则 / Decision Rules

- **红灯 (state=1)**: 发布零速度，车辆停止
- **绿灯 (state=2)**: 转发导航栈的速度命令，车辆正常行驶
- **黄灯 (state=3)**: 
  - 如果 `yellow_light_action=stop`: 停止
  - 如果 `yellow_light_action=slow`: 减速到 `slow_down_speed`
- **未检测到 (state=0)**: 转发导航栈的速度命令

## 参数调整 / Parameter Tuning

### 检测模块参数

```bash
roslaunch yolov5_ros traffic_light_integration.launch \
    confidence_threshold:=0.6  # 提高置信度阈值，减少误检
```

### 决策模块参数

在launch文件中或通过rosparam设置：

```xml
<param name="confidence_threshold" value="0.5"/>
<param name="yellow_light_action" value="stop"/>  <!-- 或 'slow' -->
<param name="slow_down_speed" value="0.2"/>
```

## 常见问题排查 / Troubleshooting

### 问题1: 检测不到红绿灯

**检查项**:
- 模型路径是否正确
- 摄像头话题是否正确
- 类别名称是否匹配（检查 `detect_ros.py` 中的 `traffic_light_classes`）

**解决方案**:
```bash
# 查看检测节点日志
rosnode info /yolov5_traffic_light_detection

# 查看图像话题
rostopic echo /camera/color/image_raw --noarr
```

### 问题2: 车辆在绿灯时也不动

**检查项**:
- `/cmd_vel_nav` 话题是否有数据
- 决策模块是否正常运行

**解决方案**:
```bash
# 检查导航栈是否发布速度命令
rostopic echo /cmd_vel_nav

# 检查决策模块日志
rosnode info /traffic_light_decision
```

### 问题3: 类别名称不匹配

如果你的模型使用的类别名称不同，修改 `detect_ros.py`:

```python
traffic_light_classes = {
    'your_red_class_name': 1,
    'your_green_class_name': 2,
    'your_yellow_class_name': 3,
}
```

## 下一步 / Next Steps

1. 根据实际场景调整参数
2. 优化模型以提高检测精度
3. 添加距离估计功能
4. 与高精地图集成

## 更多信息 / More Information

详细文档请参考: `TRAFFIC_LIGHT_INTEGRATION.md`



