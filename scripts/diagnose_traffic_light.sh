#!/bin/bash
# 红绿灯检测节点诊断脚本
# Traffic Light Detection Node Diagnosis Script

echo "=========================================="
echo "红绿灯检测节点诊断"
echo "Traffic Light Detection Node Diagnosis"
echo "=========================================="

echo -e "\n[1] 检查节点状态"
echo "摄像头节点:"
rosnode list | grep camera || echo "  ✗ 摄像头节点未运行"

echo -e "\n红绿灯相关节点:"
rosnode list | grep -E "(yolov5|traffic_light|visualizer)" || echo "  ✗ 红绿灯节点未运行"

echo -e "\n[2] 检查话题"
echo "摄像头话题:"
rostopic list | grep camera
echo -e "\n检测相关话题:"
rostopic list | grep -E "(detection|traffic_light)" || echo "  ✗ 未找到检测话题"

echo -e "\n[3] 检查摄像头数据流"
echo "检查 /camera/color/image_raw:"
timeout 2 rostopic echo /camera/color/image_raw --noarr -n 1 2>&1 | head -5 || echo "  ✗ 摄像头话题无数据"

echo -e "\n[4] 检查ROS包"
echo "yolov5_ros包:"
rospack find yolov5_ros 2>&1 || echo "  ✗ yolov5_ros包未找到"

echo -e "\n[5] 检查模型文件"
YOLOV5_PATH=$(rospack find yolov5_ros 2>/dev/null)
if [ -n "$YOLOV5_PATH" ]; then
    echo "模型文件位置: $YOLOV5_PATH/scripts/"
    ls -lh $YOLOV5_PATH/scripts/yolov5*.pt 2>/dev/null || echo "  ✗ 模型文件未找到"
else
    echo "  ✗ 无法检查模型文件（包未找到）"
fi

echo -e "\n[6] 检查日志文件"
LATEST_LOG=$(ls -td ~/.ros/log/* 2>/dev/null | head -1)
if [ -n "$LATEST_LOG" ]; then
    echo "最新日志目录: $LATEST_LOG"
    echo -e "\n红绿灯检测节点日志:"
    find $LATEST_LOG -name "*yolov5_traffic_light_detection*.log" -exec tail -30 {} \; 2>/dev/null || echo "  ✗ 未找到日志文件"
    echo -e "\n决策节点日志:"
    find $LATEST_LOG -name "*smart_traffic_light_decision*.log" -exec tail -30 {} \; 2>/dev/null || echo "  ✗ 未找到日志文件"
    echo -e "\n可视化节点日志:"
    find $LATEST_LOG -name "*traffic_light_visualizer*.log" -exec tail -30 {} \; 2>/dev/null || echo "  ✗ 未找到日志文件"
else
    echo "  ✗ 未找到日志目录"
fi

echo -e "\n[7] 检查Python依赖"
echo "检查ultralytics:"
python3 -c "import ultralytics; print('  ✓ ultralytics:', ultralytics.__version__)" 2>&1 || echo "  ✗ ultralytics未安装"

echo -e "\n检查torch:"
python3 -c "import torch; print('  ✓ torch:', torch.__version__); print('  ✓ CUDA available:', torch.cuda.is_available())" 2>&1 || echo "  ✗ torch未安装"

echo -e "\n[8] 检查节点脚本权限"
YOLOV5_PATH=$(rospack find yolov5_ros 2>/dev/null)
if [ -n "$YOLOV5_PATH" ]; then
    echo "检查脚本权限:"
    ls -l $YOLOV5_PATH/scripts/*.py 2>/dev/null | head -5
fi

echo -e "\n=========================================="
echo "诊断完成"
echo "=========================================="

