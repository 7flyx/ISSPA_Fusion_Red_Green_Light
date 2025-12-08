# 红绿灯检测模型训练完整指南
# Complete Guide for Traffic Light Detection Model Training

## 📁 文件说明 / File Description

### 核心训练文件 / Core Training Files

| 文件 | 说明 |
|------|------|
| `train.py` | YOLOv5训练主脚本（已存在） |
| `train_traffic_light.sh` | 快速训练脚本（一键训练） |
| `data/traffic_lights.yaml` | 红绿灯数据集配置文件 |

### 数据准备工具 / Data Preparation Tools

| 文件 | 说明 |
|------|------|
| `prepare_traffic_light_dataset.py` | 数据集准备和验证脚本 |
| `convert_annotation.py` | 标注格式转换工具（VOC/COCO → YOLO） |

### 辅助脚本 / Utility Scripts

| 文件 | 说明 |
|------|------|
| `download_pretrained_weights.sh` | 下载预训练权重脚本 |
| `val.py` | 模型验证脚本（已存在） |
| `detect.py` | 单张图像/视频检测脚本（已存在） |

### 文档 / Documentation

| 文件 | 说明 |
|------|------|
| `TRAIN_TRAFFIC_LIGHT.md` | 完整训练指南（详细） |
| `QUICK_TRAIN_GUIDE.md` | 快速开始指南（5分钟） |
| `README_TRAINING.md` | 本文件（总览） |

---

## 🚀 快速开始流程 / Quick Start Workflow

### 1. 准备数据

```bash
# 创建数据集目录
python prepare_traffic_light_dataset.py --mode create --dataset_path ../datasets/traffic_lights

# 将图像和标签放入对应目录，或使用自动分割
python prepare_traffic_light_dataset.py --mode split \
    --source_images /path/to/images \
    --source_labels /path/to/labels \
    --dataset_path ../datasets/traffic_lights
```

### 2. 下载预训练权重

```bash
./download_pretrained_weights.sh
# 或手动下载
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
```

### 3. 开始训练

```bash
# 方法1: 使用脚本（推荐）
./train_traffic_light.sh

# 方法2: 直接使用Python
python train.py \
    --data data/traffic_lights.yaml \
    --weights yolov5s.pt \
    --img 640 \
    --batch-size 16 \
    --epochs 100 \
    --device 0
```

### 4. 使用训练好的模型

```bash
# 复制最佳模型
cp runs/train/traffic_lights_yolov5s/weights/best.pt yolov5s_traffic_lights.pt

# 测试模型
python detect.py \
    --weights yolov5s_traffic_lights.pt \
    --source path/to/test/image.jpg \
    --img 640 \
    --conf-thres 0.5
```

---

## 📊 数据集格式 / Dataset Format

### 目录结构

```
traffic_lights/
├── images/
│   ├── train/      # 训练图像
│   ├── val/         # 验证图像
│   └── test/        # 测试图像（可选）
└── labels/
    ├── train/      # 训练标签（YOLO格式）
    ├── val/         # 验证标签
    └── test/        # 测试标签
```

### YOLO标注格式

每个图像对应一个`.txt`文件，每行一个目标：

```
class_id center_x center_y width height
```

- `class_id`: 0=red, 1=green, 2=yellow
- 坐标值归一化到[0,1]

**示例**:
```
0 0.5 0.5 0.2 0.3
1 0.3 0.4 0.15 0.25
```

---

## 🛠️ 工具使用 / Tool Usage

### 数据集准备工具

```bash
# 创建目录结构
python prepare_traffic_light_dataset.py --mode create

# 自动分割数据集
python prepare_traffic_light_dataset.py --mode split \
    --source_images /path/to/images \
    --source_labels /path/to/labels

# 验证标签文件
python prepare_traffic_light_dataset.py --mode validate \
    --labels_dir ../datasets/traffic_lights/labels/train
```

### 标注格式转换

```bash
# VOC XML → YOLO
python convert_annotation.py \
    --format voc \
    --input /path/to/voc/annotations \
    --output /path/to/yolo/labels \
    --class-mapping "red:0,green:1,yellow:2"

# COCO JSON → YOLO
python convert_annotation.py \
    --format coco \
    --input /path/to/annotations.json \
    --output /path/to/yolo/labels \
    --images /path/to/images \
    --class-mapping "red:0,green:1,yellow:2"
```

---

## 📈 训练参数说明 / Training Parameters

### 基础参数

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `--data` | 数据集配置 | `data/traffic_lights.yaml` |
| `--weights` | 预训练权重 | `yolov5s.pt` |
| `--img` | 输入尺寸 | `640` |
| `--batch-size` | 批次大小 | `16` (根据GPU调整) |
| `--epochs` | 训练轮数 | `100-200` |
| `--device` | 训练设备 | `0` (GPU) |

### 模型选择

- **yolov5n.pt**: 最快，适合实时检测
- **yolov5s.pt**: 平衡速度和精度（推荐）
- **yolov5m.pt**: 更高精度
- **yolov5l.pt**: 最高精度

---

## 📚 推荐学习资源 / Recommended Resources

### 数据集

1. **公开数据集**:
   - [Bosch Small Traffic Lights](https://hci.iwr.uni-heidelberg.de/node/6132)
   - [LISA Traffic Light Dataset](https://www.kaggle.com/datasets/mbornoe/lisa-traffic-light-dataset)
   - [BDD100K](https://www.bdd100k.com/)

2. **标注工具**:
   - [LabelImg](https://github.com/heartexlabs/labelImg) - 本地标注工具
   - [Roboflow](https://roboflow.com/) - 在线标注平台
   - [CVAT](https://github.com/openvinotoolkit/cvat) - 企业级标注工具

### 文档

- [YOLOv5官方文档](https://docs.ultralytics.com/)
- [YOLOv5 GitHub](https://github.com/ultralytics/yolov5)
- [自定义数据集训练教程](https://docs.ultralytics.com/yolov5/tutorials/train_custom_data)

---

## ❓ 常见问题 / FAQ

### Q1: 需要多少数据？

**A**: 最少每个类别50-100张，建议每个类别200-500张，总共500-1500张。

### Q2: 训练需要多长时间？

**A**: 
- 单GPU (GTX 1080Ti): 约2-4小时（100 epochs）
- CPU: 可能需要10-20小时

### Q3: 显存不足怎么办？

**A**: 
- 减小 `--batch-size` (如改为8或4)
- 减小 `--img` 尺寸 (如改为416)
- 使用更小的模型 (yolov5n)

### Q4: 如何提高检测精度？

**A**:
1. 增加训练数据
2. 提高数据质量（更好的标注）
3. 使用更大的模型
4. 增加训练轮数
5. 使用数据增强

### Q5: 训练好的模型在哪里？

**A**: 
- 最佳模型: `runs/train/traffic_lights_yolov5s/weights/best.pt`
- 最新模型: `runs/train/traffic_lights_yolov5s/weights/last.pt`

---

## 🔗 相关文档 / Related Documentation

- **集成指南**: `../TRAFFIC_LIGHT_INTEGRATION.md`
- **快速开始**: `QUICK_TRAIN_GUIDE.md`
- **详细训练指南**: `TRAIN_TRAFFIC_LIGHT.md`

---

## 📝 训练检查清单 / Training Checklist

- [ ] 收集/准备图像数据（至少500张）
- [ ] 标注数据（YOLO格式）
- [ ] 创建数据集目录结构
- [ ] 验证标注文件格式
- [ ] 下载预训练权重
- [ ] 配置数据集YAML文件
- [ ] 开始训练
- [ ] 监控训练过程
- [ ] 验证模型性能
- [ ] 测试模型
- [ ] 复制模型到项目目录
- [ ] 集成到ISSPA系统

---

## 🎯 下一步 / Next Steps

训练完成后：
1. ✅ 将模型复制到项目目录
2. ✅ 按照 `TRAFFIC_LIGHT_INTEGRATION.md` 集成到ISSPA
3. ✅ 在实际场景中测试和调优
4. ✅ 根据实际效果调整参数

---

**祝训练顺利！** 🚀



