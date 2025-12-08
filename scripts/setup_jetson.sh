#!/bin/bash
# Jetson Orin Nano Super 4GB 环境配置脚本
# Jetson Orin Nano Super 4GB Environment Setup Script

set -e

echo "=========================================="
echo "Jetson Orin Nano Super 4GB 环境配置"
echo "Jetson Orin Nano Super 4GB Environment Setup"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否为root
if [ "$EUID" -eq 0 ]; then 
    echo -e "${RED}请不要使用root用户运行此脚本${NC}"
    exit 1
fi

# 1. 系统信息检查
echo -e "\n${GREEN}[1/8] 检查系统信息...${NC}"
echo "JetPack版本:"
cat /etc/nv_tegra_release 2>/dev/null || echo "无法读取JetPack版本"

echo -e "\nCUDA版本:"
nvcc --version 2>/dev/null || echo "CUDA未安装"

echo -e "\nGPU信息:"
nvidia-smi 2>/dev/null || echo "无法获取GPU信息"

# 2. 设置性能模式
echo -e "\n${GREEN}[2/8] 设置性能模式...${NC}"
if command -v nvpmodel &> /dev/null; then
    echo "设置最大性能模式..."
    sudo nvpmodel -m 0
    sudo jetson_clocks
    echo -e "${GREEN}✓ 性能模式已设置${NC}"
else
    echo -e "${YELLOW}⚠ nvpmodel未找到，跳过性能模式设置${NC}"
fi

# 3. 更新系统
echo -e "\n${GREEN}[3/8] 更新系统包...${NC}"
read -p "是否更新系统包? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo apt-get update
    sudo apt-get upgrade -y
    echo -e "${GREEN}✓ 系统更新完成${NC}"
else
    echo -e "${YELLOW}⚠ 跳过系统更新${NC}"
fi

# 4. 安装基础工具
echo -e "\n${GREEN}[4/8] 安装基础工具...${NC}"
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    htop \
    python3-pip \
    python3-dev

echo -e "${GREEN}✓ 基础工具安装完成${NC}"

# 5. 检查ROS
echo -e "\n${GREEN}[5/8] 检查ROS安装...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}⚠ ROS未安装或未配置${NC}"
    echo "请先安装ROS:"
    echo "  Ubuntu 20.04: ROS Noetic"
    echo "  Ubuntu 22.04: ROS 2 Humble"
    read -p "是否现在安装ROS Noetic? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "安装ROS Noetic..."
        # 这里可以添加ROS安装命令
        echo -e "${YELLOW}请手动安装ROS，参考文档${NC}"
    fi
else
    echo -e "${GREEN}✓ ROS已安装: $ROS_DISTRO${NC}"
fi

# 6. 检查PyTorch
echo -e "\n${GREEN}[6/8] 检查PyTorch安装...${NC}"
python3 << EOF
import sys
try:
    import torch
    print(f"✓ PyTorch版本: {torch.__version__}")
    if torch.cuda.is_available():
        print(f"✓ CUDA可用: {torch.version.cuda}")
        print(f"✓ GPU设备: {torch.cuda.get_device_name(0)}")
    else:
        print("⚠ CUDA不可用")
except ImportError:
    print("⚠ PyTorch未安装")
    print("请安装PyTorch for Jetson:")
    print("  https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048")
    sys.exit(1)
EOF

# 7. 检查工作空间
echo -e "\n${GREEN}[7/8] 检查ROS工作空间...${NC}"
if [ -d "$HOME/catkin_ws" ]; then
    echo -e "${GREEN}✓ 工作空间已存在: ~/catkin_ws${NC}"
    read -p "是否重新编译工作空间? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cd ~/catkin_ws
        echo "编译工作空间（使用单线程以避免内存不足）..."
        catkin_make -j1
        echo -e "${GREEN}✓ 工作空间编译完成${NC}"
    fi
else
    echo -e "${YELLOW}⚠ 工作空间不存在${NC}"
    read -p "是否创建工作空间? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        mkdir -p ~/catkin_ws/src
        cd ~/catkin_ws
        catkin_make
        echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
        echo -e "${GREEN}✓ 工作空间创建完成${NC}"
    fi
fi

# 8. 安装Python依赖
echo -e "\n${GREEN}[8/8] 安装Python依赖...${NC}"
pip3 install --upgrade pip
pip3 install \
    ultralytics \
    opencv-python \
    pillow \
    matplotlib \
    scipy \
    tqdm \
    pyyaml \
    requests \
    numpy

echo -e "${GREEN}✓ Python依赖安装完成${NC}"

# 9. 创建优化配置
echo -e "\n${GREEN}创建Jetson优化配置...${NC}"
cat > ~/jetson_config.yaml << EOF
# Jetson Orin Nano Super 4GB 优化配置
device: 0
half: true
imgsz: 416
batch_size: 1
workers: 2
model: yolov5n.pt
EOF

echo -e "${GREEN}✓ 配置文件已创建: ~/jetson_config.yaml${NC}"

# 完成
echo -e "\n${GREEN}=========================================="
echo "环境配置完成！"
echo "Environment Setup Complete!"
echo "==========================================${NC}"

echo -e "\n${YELLOW}下一步:${NC}"
echo "1. 检查摄像头: ls -l /dev/video*"
echo "2. 测试YOLOv5: cd ~/catkin_ws/src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts"
echo "3. 运行: python3 detect.py --weights yolov5n.pt --source 0 --img 416 --half"
echo "4. 启动完整系统: roslaunch navigation_stack pavs_navigation.launch"

echo -e "\n${YELLOW}性能监控:${NC}"
echo "- GPU监控: watch -n 1 nvidia-smi"
echo "- 系统监控: jtop (需要安装: sudo pip3 install -U jetson-stats)"

