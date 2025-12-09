# 快速启动命令
# Quick Start Commands

## 完整启动命令（单行）

```bash
roslaunch navigation_stack pavs_navigation.launch enable_traffic_light:=true enable_camera:=true enable_visualization:=true camera_topic:=/camera/color/image_raw use_rviz:=true
```

## 其他常用命令

### 不启用可视化

```bash
roslaunch navigation_stack pavs_navigation.launch enable_traffic_light:=true enable_camera:=true enable_visualization:=false camera_topic:=/camera/color/image_raw use_rviz:=true
```

### 使用Jetson优化模式

```bash
roslaunch navigation_stack pavs_navigation.launch enable_traffic_light:=true enable_camera:=true enable_visualization:=true camera_topic:=/camera/color/image_raw use_rviz:=true jetson_mode:=true model_size:=n
```

### 不使用智能转向

```bash
roslaunch navigation_stack pavs_navigation.launch enable_traffic_light:=true enable_camera:=true enable_visualization:=true camera_topic:=/camera/color/image_raw use_rviz:=true use_smart_turn:=false
```

