#!/bin/bash
# Jetson平台ultralytics安装脚本
# Install ultralytics on Jetson Platform

set -e

echo "=========================================="
echo "在Jetson上安装ultralytics"
echo "Installing ultralytics on Jetson"
echo "=========================================="

# 方案1: 尝试安装旧版本（不依赖polars）
echo -e "\n[方案1] 尝试安装ultralytics 8.0.x版本..."
pip3 install "ultralytics<8.3.0" || {
    echo "方案1失败，尝试方案2..."
    
    # 方案2: 安装基础依赖后安装ultralytics
    echo -e "\n[方案2] 安装基础依赖..."
    pip3 install pyyaml scipy matplotlib requests psutil opencv-python || true
    
    echo -e "\n[方案2] 安装ultralytics（跳过可选依赖）..."
    pip3 install ultralytics --no-deps || {
        echo "方案2失败，尝试方案3..."
        
        # 方案3: 使用pip的--no-build-isolation
        echo -e "\n[方案3] 使用--no-build-isolation安装..."
        pip3 install ultralytics --no-build-isolation || {
            echo "所有方案都失败，使用最小安装..."
            
            # 方案4: 最小安装
            echo -e "\n[方案4] 最小安装（仅核心功能）..."
            pip3 install pyyaml scipy matplotlib requests psutil
            echo "注意: ultralytics可能无法完全安装，但代码有fallback机制"
        }
    }
}

# 验证安装
echo -e "\n验证安装..."
python3 << EOF
try:
    import ultralytics
    print(f"✓ ultralytics installed: {ultralytics.__version__}")
except ImportError:
    try:
        # 检查是否有fallback可用
        import sys
        sys.path.insert(0, '/home/jetson/ISSPA2/ISSPA_Fusion_Red_Green_Light/src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts')
        from utils.plots import Annotator, colors
        print("✓ Using local fallback implementation")
    except ImportError:
        print("✗ ultralytics not installed and no fallback available")
        print("请手动安装: pip3 install 'ultralytics<8.3.0'")
EOF

echo -e "\n安装完成！"
echo "如果仍有问题，请参考: docs/JETSON_ULTRALYTICS_FIX.md"

