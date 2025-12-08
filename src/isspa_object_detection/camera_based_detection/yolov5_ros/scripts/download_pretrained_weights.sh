#!/bin/bash
# 下载YOLOv5预训练权重
# Download YOLOv5 Pretrained Weights

echo "Downloading YOLOv5 pretrained weights..."

# 创建weights目录
mkdir -p weights
cd weights

# 下载不同大小的模型
models=("yolov5n.pt" "yolov5s.pt" "yolov5m.pt" "yolov5l.pt" "yolov5x.pt")

for model in "${models[@]}"; do
    if [ ! -f "$model" ]; then
        echo "Downloading $model..."
        wget https://github.com/ultralytics/yolov5/releases/download/v7.0/$model
    else
        echo "$model already exists, skipping..."
    fi
done

echo ""
echo "All pretrained weights downloaded!"
echo "Available models:"
ls -lh *.pt



