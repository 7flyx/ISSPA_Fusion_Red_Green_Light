#!/bin/bash
# 在PC上运行：安装最小化包
# Run on PC: Install minimal package

set -e

echo "=========================================="
echo "PC端最小化安装"
echo "PC Minimal Installation"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未设置"
    echo "请先运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo -e "\nROS版本: $ROS_DISTRO"

# 创建工作空间
WORKSPACE="${HOME}/catkin_ws"
if [ ! -d "$WORKSPACE" ]; then
    echo -e "\n[1/5] 创建工作空间..."
    mkdir -p "$WORKSPACE/src"
    echo "✓ 工作空间已创建: $WORKSPACE"
else
    echo -e "\n[1/5] 工作空间已存在: $WORKSPACE"
fi

# 查找tar.gz文件
TAR_FILE=""
if [ -f "${HOME}/Desktop/isspa_pc_minimal.tar.gz" ]; then
    TAR_FILE="${HOME}/Desktop/isspa_pc_minimal.tar.gz"
elif [ -f "${HOME}/isspa_pc_minimal.tar.gz" ]; then
    TAR_FILE="${HOME}/isspa_pc_minimal.tar.gz"
else
    echo -e "\n[2/5] 查找安装包..."
    echo "未找到 isspa_pc_minimal.tar.gz"
    read -p "请输入tar.gz文件的完整路径: " TAR_FILE
fi

if [ ! -f "$TAR_FILE" ]; then
    echo "错误: 文件不存在: $TAR_FILE"
    exit 1
fi

echo -e "\n[2/5] 解压安装包..."
cd "$WORKSPACE"
tar -xzf "$TAR_FILE"
echo "✓ 解压完成"

# 检查必要文件
echo -e "\n[3/5] 检查文件..."
if [ ! -d "$WORKSPACE/src/isspa_object_detection/camera_based_detection/yolov5_ros" ]; then
    echo "✗ 错误: yolov5_ros包未找到"
    exit 1
fi
echo "✓ yolov5_ros包存在"

if [ ! -f "$WORKSPACE/src/isspa_navigation/navigation_stack/launch/pavs_navigation_pc_only.launch" ]; then
    echo "⚠ 警告: pavs_navigation_pc_only.launch 未找到"
fi

# 安装ROS依赖
echo -e "\n[4/5] 安装ROS依赖..."
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-nav-msgs \
    ros-$ROS_DISTRO-move-base-msgs \
    ros-$ROS_DISTRO-actionlib \
    ros-$ROS_DISTRO-actionlib-msgs \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-image-view || echo "⚠ 部分依赖安装失败，请手动安装"

# 安装Python依赖
echo -e "\n[5/5] 安装Python依赖..."
pip3 install --upgrade pip
pip3 install ultralytics opencv-python numpy || echo "⚠ 部分Python依赖安装失败"

# 编译工作空间
echo -e "\n编译工作空间..."
cd "$WORKSPACE"
catkin_make || {
    echo "⚠ 编译失败，尝试修复..."
    # 创建缺失的package.xml
    if [ ! -f "$WORKSPACE/src/isspa_navigation/navigation_stack/package.xml" ]; then
        cat > "$WORKSPACE/src/isspa_navigation/navigation_stack/package.xml" << 'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>navigation_stack</name>
  <version>1.0.0</version>
  <description>Navigation stack for ISSPA</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
</package>
EOF
    fi
    
    # 创建缺失的CMakeLists.txt
    if [ ! -f "$WORKSPACE/src/isspa_navigation/navigation_stack/CMakeLists.txt" ]; then
        cat > "$WORKSPACE/src/isspa_navigation/navigation_stack/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(navigation_stack)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
EOF
    fi
    
    catkin_make
}

echo -e "\n=========================================="
echo "✓ 安装完成！"
echo "=========================================="
echo -e "\n下一步:"
echo "1. Source工作空间:"
echo "   source $WORKSPACE/devel/setup.bash"
echo ""
echo "2. 添加到 ~/.bashrc:"
echo "   echo 'source $WORKSPACE/devel/setup.bash' >> ~/.bashrc"
echo ""
echo "3. 测试:"
echo "   rospack find yolov5_ros"
echo "   roslaunch navigation_stack pavs_navigation_pc_only.launch --check"

