# Jetson快速修复指南
# Jetson Quick Fix Guide

## 当前错误 / Current Errors

根据你的错误日志，有两个问题需要修复：

### 错误1: ImportError: cannot import name 'GoalID'

```
ImportError: cannot import name 'GoalID' from 'move_base_msgs.msg'
```

**已修复**: 代码已更新，`GoalID` 现在从 `actionlib_msgs.msg` 导入

### 错误2: ModuleNotFoundError: No module named 'ultralytics'

```
ModuleNotFoundError: No module named 'ultralytics'
```

**需要安装**: `ultralytics` 模块

## 快速修复步骤 / Quick Fix Steps

### 步骤1: 安装ultralytics

```bash
pip3 install ultralytics
```

### 步骤2: 安装ROS actionlib（如果未安装）

```bash
sudo apt-get install -y ros-noetic-actionlib ros-noetic-actionlib-msgs
```

### 步骤3: 重新编译工作空间

```bash
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light
catkin_make -j1
source devel/setup.bash
```

### 步骤4: 重新启动

```bash
roslaunch navigation_stack pavs_navigation.launch \
    enable_traffic_light:=true \
    enable_camera:=true \
    enable_visualization:=true \
    camera_topic:=/camera/color/image_raw
```

## 一键修复脚本 / One-Click Fix Script

创建并运行以下脚本：

```bash
cat > ~/fix_isspa.sh << 'EOF'
#!/bin/bash
echo "修复ISSPA依赖..."

# 安装ultralytics
echo "安装ultralytics..."
pip3 install ultralytics

# 安装ROS依赖
echo "安装ROS依赖..."
sudo apt-get install -y ros-noetic-actionlib ros-noetic-actionlib-msgs

# 重新编译
echo "重新编译工作空间..."
cd ~/ISSPA2/ISSPA_Fusion_Red_Green_Light
catkin_make -j1
source devel/setup.bash

echo "修复完成！"
EOF

chmod +x ~/fix_isspa.sh
~/fix_isspa.sh
```

## 验证修复 / Verify Fix

### 检查ultralytics

```bash
python3 -c "import ultralytics; print('✓ ultralytics installed')"
```

### 检查GoalID

```bash
python3 << EOF
import rospy
from actionlib_msgs.msg import GoalID
print("✓ GoalID import successful")
EOF
```

## 如果仍有问题 / If Still Having Issues

### 检查Python环境

```bash
# 检查pip3安装位置
pip3 show ultralytics

# 检查Python路径
python3 -c "import sys; print(sys.path)"
```

### 使用虚拟环境（可选）

```bash
# 创建虚拟环境
python3 -m venv ~/isspa_venv
source ~/isspa_venv/bin/activate

# 安装依赖
pip install ultralytics

# 在launch文件中使用虚拟环境的Python
```

## 摄像头问题 / Camera Issues

如果摄像头一直显示 "wait for device to be connected"：

```bash
# 检查摄像头设备
ls -l /dev/video*

# 检查USB连接
lsusb

# 测试摄像头
v4l2-ctl --list-devices
```

## 下一步 / Next Steps

修复完成后：

1. ✅ 重新编译工作空间
2. ✅ 重新启动launch文件
3. ✅ 检查所有节点是否正常运行
4. ✅ 查看可视化窗口

