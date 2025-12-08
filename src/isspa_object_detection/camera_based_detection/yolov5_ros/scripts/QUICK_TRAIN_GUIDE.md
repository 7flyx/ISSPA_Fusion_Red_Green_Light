# 红绿灯检测模型快速训练指南
# Quick Training Guide for Traffic Light Detection

## 5分钟快速开始 / 5-Minute Quick Start

### 步骤1: 准备数据（如果没有数据）

如果你还没有标注好的数据，可以：

1. **使用公开数据集**:
   - [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132)
   - [LISA Traffic Light Dataset](https://www.kaggle.com/datasets/mbornoe/lisa-traffic-light-dataset)
   - [BDD100K](https://www.bdd100k.com/) (包含红绿灯标注)

2. **自己标注**:
   - 使用LabelImg工具标注
   - 参考 `TRAIN_TRAFFIC_LIGHT.md` 中的标注说明

### 步骤2: 下载预训练权重

```bash
cd src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts
chmod +x download_pretrained_weights.sh
./download_pretrained_weights.sh
```

或者手动下载：
```bash
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
```

### 步骤3: 准备数据集

```bash
# 创建数据集目录结构
python prepare_traffic_light_dataset.py --mode create --dataset_path ../datasets/traffic_lights

# 将你的图像和标签文件放入对应目录
# 或使用自动分割
python prepare_traffic_light_dataset.py --mode split \
    --source_images /path/to/images \
    --source_labels /path/to/labels \
    --dataset_path ../datasets/traffic_lights
```

### 步骤4: 开始训练

**方法1: 使用训练脚本（推荐）**

```bash
chmod +x train_traffic_light.sh
./train_traffic_light.sh
```

**方法2: 直接使用Python命令**

```bash
python train.py \
    --data data/traffic_lights.yaml \
    --weights yolov5s.pt \
    --img 640 \
    --batch-size 16 \
    --epochs 100 \
    --device 0
```

### 步骤5: 使用训练好的模型

训练完成后，模型会保存在：
```
runs/train/traffic_lights_yolov5s/weights/best.pt
```

复制到项目目录：
```bash
cp runs/train/traffic_lights_yolov5s/weights/best.pt yolov5s_traffic_lights.pt
```

---

## 最小数据集要求 / Minimum Dataset Requirements

- **最少图像数**: 每个类别至少50-100张
- **总图像数**: 建议至少300-500张
- **训练/验证比例**: 70% / 20% / 10%

---

## 常见问题快速解决 / Quick Troubleshooting

### 问题1: 找不到数据集配置文件

**解决**: 确保 `data/traffic_lights.yaml` 存在，路径正确

### 问题2: 显存不足

**解决**: 减小batch size
```bash
python train.py --data data/traffic_lights.yaml --weights yolov5s.pt --batch-size 8
```

### 问题3: 训练很慢

**解决**: 
- 使用GPU: `--device 0`
- 减小图像尺寸: `--img 416`

### 问题4: 检测精度不高

**解决**:
- 增加训练数据
- 增加训练轮数: `--epochs 200`
- 使用更大的模型: `--weights yolov5m.pt`

---

## 完整文档 / Full Documentation

详细说明请参考: `TRAIN_TRAFFIC_LIGHT.md`



