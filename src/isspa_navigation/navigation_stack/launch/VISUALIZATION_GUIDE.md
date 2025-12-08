# 红绿灯检测可视化指南
# Traffic Light Detection Visualization Guide

## 功能说明 / Features

启动 `pavs_navigation.launch` 时，可以同时启动摄像头和可视化窗口，实时查看：

1. **红绿灯检测结果** - 检测到的红绿灯状态和置信度
2. **决策状态** - 当前决策（红灯/绿灯/黄灯）
3. **车辆状态** - 是否停止
4. **转向信息** - 转向意图和转向状态

## 使用方法 / Usage

### 启动完整系统（包含摄像头和可视化）

```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    enable_camera:=true \
    enable_visualization:=true \
    camera_topic:=/camera/color/image_raw
```

### 参数说明 / Parameters

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `enable_camera` | 是否启动摄像头 | `true` |
| `enable_visualization` | 是否启用可视化窗口 | `true` |
| `camera_name` | 摄像头名称 | `camera` |
| `camera_topic` | 摄像头图像话题 | `/camera/color/image_raw` |

### 禁用摄像头（如果摄像头已通过其他方式启动）

```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    enable_camera:=false \
    enable_visualization:=true \
    camera_topic:=/your/camera/topic
```

### 禁用可视化窗口

```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    enable_visualization:=false
```

## 可视化窗口内容 / Visualization Window Content

可视化窗口会显示以下信息：

### 1. 红绿灯检测状态
- **状态文本**: 显示检测到的红绿灯状态（红灯/绿灯/黄灯）
- **置信度**: 显示检测置信度（0-1）
- **颜色标识**: 根据状态显示不同颜色

### 2. 决策状态
- **决策结果**: 显示当前决策（红灯停止/绿灯通行/黄灯减速）
- **车辆状态**: 显示"车辆已停止"或"车辆正常行驶"

### 3. 转向信息（如果启用智能转向）
- **转向意图**: 显示当前转向意图（直行/左转/右转）
- **转向状态**: 显示转向状态机状态（normal/approaching/checking/turning/completed）

### 4. 时间戳
- 显示当前时间戳

## 摄像头配置 / Camera Configuration

### Astra摄像头

默认配置使用Astra摄像头，如果使用其他摄像头：

1. **修改摄像头启动部分**:
```xml
<!-- 在pavs_navigation.launch中 -->
<include file="$(find your_camera_package)/launch/your_camera.launch"/>
```

2. **调整摄像头话题**:
```bash
roslaunch navigation_stack pavs_navigation.launch \
    camera_topic:=/your/camera/image/topic
```

### USB摄像头

如果使用USB摄像头，可以使用 `usb_cam` 包：

```bash
# 安装usb_cam（如果未安装）
sudo apt-get install ros-<distro>-usb-cam

# 在launch文件中添加
<node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
    <param name="video_device" value="/dev/video0"/>
    <param name="image_width" value="640"/>
    <param name="image_height" value="480"/>
    <remap from="usb_cam/image_raw" to="/camera/color/image_raw"/>
</node>
```

## 故障排查 / Troubleshooting

### 问题1: 摄像头无法启动

**检查项**:
- 摄像头是否连接
- 摄像头驱动是否安装
- 摄像头权限是否正确

**解决**:
```bash
# 检查摄像头设备
ls -l /dev/video*

# 检查摄像头权限
sudo chmod 666 /dev/video0
```

### 问题2: 可视化窗口不显示

**检查项**:
- 是否启用了可视化 (`enable_visualization:=true`)
- 是否有图像数据发布到 `/detection_results`
- 是否有GUI环境（X11）

**解决**:
```bash
# 检查话题
rostopic echo /detection_results --noarr

# 检查可视化节点
rosnode info /traffic_light_visualizer
```

### 问题3: 图像显示延迟

**可能原因**:
- 检测节点处理速度慢
- 网络延迟
- 图像分辨率过高

**解决**:
- 降低图像分辨率
- 优化检测模型
- 检查系统性能

## 键盘快捷键 / Keyboard Shortcuts

可视化窗口支持以下操作：

- **关闭窗口**: 点击窗口关闭按钮或按 `q` 键（如果实现）
- **调整窗口大小**: 拖动窗口边缘

## 性能优化 / Performance Optimization

### 降低图像分辨率

在摄像头启动时降低分辨率：

```xml
<arg name="color_width" value="320"/>
<arg name="color_height" value="240"/>
```

### 降低帧率

调整检测节点的处理频率：

```xml
<param name="rate" value="5"/>  <!-- 5 Hz instead of 10 Hz -->
```

## 相关话题 / Related Topics

### 订阅话题

可视化节点订阅以下话题：

- `/detection_results` - 检测结果图像
- `/traffic_light_state` - 红绿灯状态
- `/traffic_light_decision` - 决策结果
- `/traffic_light_turn_intention` - 转向意图
- `/traffic_light_turn_state` - 转向状态
- `/traffic_light_should_stop` - 停止标志

### 查看话题数据

```bash
# 查看检测结果图像
rostopic echo /detection_results --noarr

# 查看红绿灯状态
rostopic echo /traffic_light_state

# 查看决策结果
rostopic echo /traffic_light_decision
```

## 示例截图说明 / Screenshot Description

可视化窗口布局：

```
┌─────────────────────────────────────┐
│  [信息面板 - 半透明黑色背景]          │
│  红绿灯状态: 绿灯 (GREEN)            │
│  置信度: 0.85                        │
│  决策: 绿灯 (GREEN)                  │
│  车辆正常行驶                        │
│  转向意图: STRAIGHT                  │
│  转向状态: NORMAL                    │
├─────────────────────────────────────┤
│                                     │
│        [检测结果图像]                │
│        (带检测框和标签)              │
│                                     │
│                                     │
└─────────────────────────────────────┘
时间: 1234567890.123
```

## 总结 / Summary

✅ **已实现**:
- 摄像头自动启动
- 实时可视化窗口
- 检测结果显示
- 决策状态显示
- 转向信息显示

✅ **使用方法**:
```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    enable_camera:=true \
    enable_visualization:=true
```

现在可以实时查看红绿灯检测和决策情况了！🎥

