# Jetson问题排查指南
# Jetson Troubleshooting Guide

## ultralytics安装问题 / ultralytics Installation Issues

### 问题: polars依赖安装失败

**错误信息**:
```
ERROR: Could not find a version that satisfies the requirement puccinialin
```

**原因**: 
- `polars` 包在ARM架构（Jetson）上编译困难
- 新版本的 `ultralytics` (>=8.3.0) 依赖 `polars`
- 清华镜像源可能缺少ARM架构的预编译包

### 解决方案

#### 方案1: 安装旧版本（最简单，推荐）

```bash
pip3 install "ultralytics<8.3.0"
```

这会安装8.2.x版本，不依赖polars。

#### 方案2: 使用官方PyPI源

```bash
# 临时使用官方源
pip3 install "ultralytics<8.3.0" -i https://pypi.org/simple
```

#### 方案3: 跳过polars依赖

```bash
# 先安装基础依赖
pip3 install pyyaml scipy matplotlib requests psutil opencv-python

# 安装ultralytics但跳过polars
pip3 install ultralytics --no-deps
pip3 install pyyaml scipy matplotlib requests psutil
```

#### 方案4: 不使用ultralytics（代码已支持）

代码已经添加了fallback机制，即使不安装ultralytics也能运行（功能可能受限）。

## 验证安装 / Verify Installation

```bash
# 检查ultralytics
python3 -c "import ultralytics; print('Version:', ultralytics.__version__)"

# 如果失败，检查fallback
python3 << EOF
import sys
sys.path.insert(0, '/home/jetson/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts')
try:
    from utils.plots import Annotator, colors
    print("Using local implementation")
except:
    print("Need to install ultralytics or fix local implementation")
EOF
```

## 其他常见问题 / Other Common Issues

### 问题1: PyTorch版本不兼容

```bash
# 检查PyTorch版本
python3 -c "import torch; print(torch.__version__)"

# 如果版本不对，重新安装PyTorch for Jetson
# 参考: docs/JETSON_ORIN_SETUP.md
```

### 问题2: 显存不足

```bash
# 使用更小的模型
# 在launch文件中设置: model_size:=n

# 降低分辨率
# 在launch文件中设置: jetson_mode:=true (自动使用416x416)
```

### 问题3: 编译内存不足

```bash
# 使用单线程编译
catkin_make -j1

# 增加swap空间
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## 快速修复脚本 / Quick Fix Script

```bash
# 在Jetson上运行
cd ~
wget https://raw.githubusercontent.com/your-repo/scripts/install_ultralytics_jetson.sh
chmod +x install_ultralytics_jetson.sh
./install_ultralytics_jetson.sh
```

