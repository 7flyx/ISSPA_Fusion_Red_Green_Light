#!/bin/bash
# 在Jetson上运行：创建PC端最小化安装包
# Run on Jetson: Create minimal package for PC installation

set -e

echo "=========================================="
echo "创建PC端最小化安装包"
echo "Create Minimal Package for PC"
echo "=========================================="

# 检查项目路径
PROJECT_ROOT="${HOME}/ISSPA2/ISSPA_Fusion_Red_Green_Light"
if [ ! -d "$PROJECT_ROOT" ]; then
    echo "错误: 找不到项目目录: $PROJECT_ROOT"
    echo "请修改脚本中的PROJECT_ROOT变量"
    exit 1
fi

# 创建临时目录
TEMP_DIR="${HOME}/isspa_pc_minimal_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEMP_DIR/src/isspa_object_detection/camera_based_detection"
mkdir -p "$TEMP_DIR/src/isspa_navigation/navigation_stack/launch"

echo -e "\n[1/4] 复制yolov5_ros包..."
cp -r "$PROJECT_ROOT/src/isspa_object_detection/camera_based_detection/yolov5_ros" \
    "$TEMP_DIR/src/isspa_object_detection/camera_based_detection/"

echo "[2/4] 复制launch文件..."
cp "$PROJECT_ROOT/src/isspa_navigation/navigation_stack/launch/pavs_navigation_pc_only.launch" \
    "$TEMP_DIR/src/isspa_navigation/navigation_stack/launch/" 2>/dev/null || echo "  ⚠ pavs_navigation_pc_only.launch 未找到"

cp "$PROJECT_ROOT/src/isspa_navigation/navigation_stack/launch/pavs_navigation_vehicle_only.launch" \
    "$TEMP_DIR/src/isspa_navigation/navigation_stack/launch/" 2>/dev/null || echo "  ⚠ pavs_navigation_vehicle_only.launch 未找到"

# 创建package.xml（如果需要）
echo "[3/4] 创建package.xml..."
cat > "$TEMP_DIR/src/isspa_navigation/navigation_stack/package.xml" << 'EOF'
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

# 创建CMakeLists.txt（如果需要）
cat > "$TEMP_DIR/src/isspa_navigation/navigation_stack/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(navigation_stack)
find_package(catkin REQUIRED COMPONENTS rospy)
catkin_package()
EOF

echo "[4/4] 打包..."
cd "$TEMP_DIR"
tar -czf "${HOME}/isspa_pc_minimal.tar.gz" src/

echo -e "\n=========================================="
echo "✓ 打包完成！"
echo "=========================================="
echo -e "\n文件位置: ${HOME}/isspa_pc_minimal.tar.gz"
echo -e "\n下一步:"
echo "1. 将文件传输到PC:"
echo "   scp ${HOME}/isspa_pc_minimal.tar.gz flyx@<PC_IP>:~/Desktop/"
echo ""
echo "2. 在PC上解压:"
echo "   cd ~/catkin_ws"
echo "   tar -xzf ~/Desktop/isspa_pc_minimal.tar.gz"
echo "   catkin_make"
echo "   source devel/setup.bash"

# 清理临时目录
read -p "是否删除临时目录? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    rm -rf "$TEMP_DIR"
    echo "✓ 临时目录已删除"
else
    echo "临时目录保留在: $TEMP_DIR"
fi

