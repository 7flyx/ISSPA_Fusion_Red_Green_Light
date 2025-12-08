#!/bin/bash
# 红绿灯检测模型训练脚本
# Traffic Light Detection Model Training Script

# 设置参数
DATA_CONFIG="data/traffic_lights.yaml"
WEIGHTS="yolov5s.pt"  # 从预训练模型开始，或使用 '' 从零开始
IMG_SIZE=640
BATCH_SIZE=16
EPOCHS=100
DEVICE=0  # GPU设备ID，使用 'cpu' 表示CPU训练
WORKERS=8
PROJECT_NAME="traffic_lights_yolov5s"

# 检查数据集是否存在
if [ ! -f "$DATA_CONFIG" ]; then
    echo "Error: Dataset config file not found: $DATA_CONFIG"
    echo "Please create the dataset first using prepare_traffic_light_dataset.py"
    exit 1
fi

# 检查预训练权重是否存在
if [ "$WEIGHTS" != "" ] && [ ! -f "$WEIGHTS" ]; then
    echo "Downloading pretrained weights..."
    python -c "from utils.downloads import attempt_download; attempt_download('$WEIGHTS')"
fi

# 开始训练
echo "Starting training..."
echo "Data config: $DATA_CONFIG"
echo "Weights: $WEIGHTS"
echo "Image size: $IMG_SIZE"
echo "Batch size: $BATCH_SIZE"
echo "Epochs: $EPOCHS"
echo "Device: $DEVICE"
echo ""

python train.py \
    --data $DATA_CONFIG \
    --weights $WEIGHTS \
    --img $IMG_SIZE \
    --batch-size $BATCH_SIZE \
    --epochs $EPOCHS \
    --device $DEVICE \
    --workers $WORKERS \
    --name $PROJECT_NAME \
    --hyp data/hyps/hyp.scratch-med.yaml

# 训练完成后，复制最佳模型
if [ -f "runs/train/$PROJECT_NAME/weights/best.pt" ]; then
    echo ""
    echo "Training completed! Best model saved at:"
    echo "runs/train/$PROJECT_NAME/weights/best.pt"
    echo ""
    echo "To use this model, copy it to:"
    echo "yolov5s_traffic_lights.pt"
    echo ""
    read -p "Copy best model to yolov5s_traffic_lights.pt? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cp runs/train/$PROJECT_NAME/weights/best.pt yolov5s_traffic_lights.pt
        echo "Model copied to yolov5s_traffic_lights.pt"
    fi
else
    echo "Error: Training failed or model not found"
    exit 1
fi



