# 红绿灯检测集成说明
# Traffic Light Detection Integration Guide

## 文件说明 / File Description

- `pavs_navigation.launch` - **已修改**：集成了红绿灯检测和决策功能
- `pavs_navigation.launch.bak` - **备份文件**：原始版本，未集成红绿灯功能

## 使用方法 / Usage

### 启动带红绿灯检测的导航系统

```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    use_smart_turn:=true \
    camera_topic:=/camera/color/image_raw \
    map:=your_map_name
```

### 参数说明 / Parameters

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `enable_traffic_light` | 是否启用红绿灯检测 | `true` |
| `use_smart_turn` | 是否使用智能转向决策 | `true` |
| `camera_topic` | 摄像头图像话题 | `/camera/color/image_raw` |
| `traffic_light_model` | 红绿灯检测模型路径 | `yolov5s_traffic_lights.pt` |
| `confidence_threshold` | 检测置信度阈值 | `0.5` |

### 禁用红绿灯检测

如果不想使用红绿灯检测功能：

```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=false
```

### 使用基础决策模块（不使用智能转向）

```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    use_smart_turn:=false
```

## 功能说明 / Features

### 启用红绿灯检测时

1. **红绿灯检测节点**: 使用YOLOv5检测图像中的红绿灯
2. **决策节点**: 根据红绿灯状态做出决策
3. **速度命令融合**: 将导航栈的速度命令与红绿灯决策融合

### 智能转向决策（use_smart_turn=true）

- ✅ 分阶段转向决策
- ✅ 转向点识别
- ✅ 障碍物检测（需要激光雷达）
- ✅ 安全确认机制

### 基础决策模块（use_smart_turn=false）

- ✅ 基础红绿灯决策
- ✅ 红灯停止、绿灯通行
- ✅ 不包含转向点识别和障碍物检测

## 必需话题 / Required Topics

### 启用智能转向时

1. `/camera/color/image_raw` - 摄像头图像
2. `/scan` - 激光雷达数据（用于障碍物检测）
3. `/current_pose` - 车辆位姿（或根据实际情况调整）
4. `/move_base/NavfnROS/plan` - 导航路径（或根据实际情况调整）

### 仅启用基础决策时

1. `/camera/color/image_raw` - 摄像头图像

## 话题重映射 / Topic Remapping

如果话题名称不同，可以在launch文件中添加重映射：

```xml
<remap from="/current_pose" to="/your/pose/topic"/>
<remap from="/scan" to="/your/laser/topic"/>
```

## 恢复原始版本 / Restore Original Version

如果需要恢复到原始版本（不包含红绿灯功能）：

```bash
cp pavs_navigation.launch.bak pavs_navigation.launch
```

## 注意事项 / Notes

1. **模型文件**: 确保红绿灯检测模型文件存在于指定路径
2. **摄像头话题**: 根据实际摄像头话题调整 `camera_topic` 参数
3. **激光雷达**: 使用智能转向时需要激光雷达数据
4. **位姿话题**: 如果车辆位姿话题不是 `/current_pose`，需要修改代码或重映射

## 相关文档 / Related Documentation

- `../yolov5_ros/TRAFFIC_LIGHT_INTEGRATION.md` - 红绿灯集成详细指南
- `../yolov5_ros/SMART_TURN_DECISION.md` - 智能转向决策说明
- `../yolov5_ros/TURN_TRAFFIC_LIGHT_GUIDE.md` - 转向红绿灯指南

