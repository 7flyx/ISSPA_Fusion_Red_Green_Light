# 红绿灯检测模型训练指南
# Traffic Light Detection Model Training Guide

## 目录 / Table of Contents

1. [数据集准备](#数据集准备)
2. [数据标注](#数据标注)
3. [训练模型](#训练模型)
4. [验证和测试](#验证和测试)
5. [模型导出](#模型导出)

---

## 数据集准备 / Dataset Preparation

### 1. 收集图像数据

收集包含红绿灯的图像，建议：
- **数量**: 至少500-1000张图像（每个类别至少100-200张）
- **多样性**: 不同光照条件、天气、角度、距离
- **分辨率**: 建议640x640或更高
- **格式**: JPG或PNG

### 2. 创建数据集目录结构

使用提供的脚本创建目录结构：

```bash
cd src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts
python prepare_traffic_light_dataset.py --mode create --dataset_path ../datasets/traffic_lights
```

目录结构：
```
traffic_lights/
├── images/
│   ├── train/      # 训练图像
│   ├── val/        # 验证图像
│   └── test/       # 测试图像（可选）
└── labels/
    ├── train/      # 训练标签
    ├── val/        # 验证标签
    └── test/       # 测试标签（可选）
```

### 3. 自动分割数据集

如果你有所有图像在一个目录中：

```bash
python prepare_traffic_light_dataset.py --mode split \
    --source_images /path/to/all/images \
    --source_labels /path/to/all/labels \
    --dataset_path ../datasets/traffic_lights \
    --train_ratio 0.7 \
    --val_ratio 0.2 \
    --test_ratio 0.1
```

---

## 数据标注 / Data Annotation

### 标注格式

YOLOv5使用YOLO格式的标注文件（.txt），每行一个目标：

```
class_id center_x center_y width height
```

- **class_id**: 类别ID（0=red, 1=green, 2=yellow）
- **center_x, center_y**: 边界框中心点坐标（归一化到0-1）
- **width, height**: 边界框宽度和高度（归一化到0-1）

### 标注工具推荐

#### 1. LabelImg（推荐）

**安装**:
```bash
pip install labelimg
# 或
conda install -c conda-forge labelimg
```

**使用**:
1. 启动LabelImg: `labelimg`
2. 打开图像目录
3. 选择YOLO格式
4. 创建类别文件 `classes.txt`:
   ```
   red
   green
   yellow
   ```
5. 标注图像并保存

#### 2. Roboflow

在线标注工具：https://roboflow.com/
- 支持团队协作
- 自动数据增强
- 可直接导出YOLOv5格式

#### 3. CVAT

企业级标注工具：https://github.com/openvinotoolkit/cvat

### 标注示例

假设图像尺寸为640x480，红绿灯边界框为：
- 左上角: (100, 50)
- 右下角: (200, 150)

转换为YOLO格式：
```
center_x = (100 + 200) / 2 / 640 = 0.234
center_y = (50 + 150) / 2 / 480 = 0.208
width = (200 - 100) / 640 = 0.156
height = (150 - 50) / 480 = 0.208

# 如果是红灯（class_id=0）:
0 0.234 0.208 0.156 0.208
```

### 验证标注文件

```bash
python prepare_traffic_light_dataset.py --mode validate \
    --labels_dir ../datasets/traffic_lights/labels/train
```

---

## 训练模型 / Model Training

### 1. 检查数据集配置

确保 `data/traffic_lights.yaml` 配置正确：

```yaml
path: ../datasets/traffic_lights
train: images/train
val: images/val
nc: 3
names:
  0: red
  1: green
  2: yellow
```

### 2. 开始训练

#### 基础训练（从预训练模型开始，推荐）

```bash
cd src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts

python train.py \
    --data data/traffic_lights.yaml \
    --weights yolov5s.pt \
    --img 640 \
    --batch-size 16 \
    --epochs 100 \
    --name traffic_lights_yolov5s
```

#### 从零开始训练

```bash
python train.py \
    --data data/traffic_lights.yaml \
    --weights '' \
    --cfg models/yolov5s.yaml \
    --img 640 \
    --batch-size 16 \
    --epochs 200 \
    --name traffic_lights_yolov5s_scratch
```

#### 使用更大的模型（提高精度）

```bash
# YOLOv5m
python train.py \
    --data data/traffic_lights.yaml \
    --weights yolov5m.pt \
    --img 640 \
    --batch-size 8 \
    --epochs 100 \
    --name traffic_lights_yolov5m

# YOLOv5l
python train.py \
    --data data/traffic_lights.yaml \
    --weights yolov5l.pt \
    --img 640 \
    --batch-size 4 \
    --epochs 100 \
    --name traffic_lights_yolov5l
```

### 3. 训练参数说明

| 参数 | 说明 | 推荐值 |
|------|------|--------|
| `--data` | 数据集配置文件 | `data/traffic_lights.yaml` |
| `--weights` | 预训练权重 | `yolov5s.pt` (从预训练开始) |
| `--img` | 输入图像尺寸 | `640` |
| `--batch-size` | 批次大小 | `16` (根据GPU内存调整) |
| `--epochs` | 训练轮数 | `100-200` |
| `--device` | 训练设备 | `0` (GPU) 或 `cpu` |
| `--workers` | 数据加载线程数 | `8` |
| `--hyp` | 超参数文件 | `data/hyps/hyp.scratch-med.yaml` |

### 4. 多GPU训练

```bash
python -m torch.distributed.run \
    --nproc_per_node 2 \
    train.py \
    --data data/traffic_lights.yaml \
    --weights yolov5s.pt \
    --img 640 \
    --batch-size 32 \
    --epochs 100 \
    --device 0,1
```

### 5. 继续训练（Resume）

如果训练中断，可以从检查点继续：

```bash
python train.py \
    --data data/traffic_lights.yaml \
    --weights runs/train/traffic_lights_yolov5s/weights/last.pt \
    --resume
```

---

## 验证和测试 / Validation and Testing

### 1. 验证模型

训练过程中会自动验证，也可以手动验证：

```bash
python val.py \
    --data data/traffic_lights.yaml \
    --weights runs/train/traffic_lights_yolov5s/weights/best.pt \
    --img 640
```

### 2. 测试单张图像

```bash
python detect.py \
    --weights runs/train/traffic_lights_yolov5s/weights/best.pt \
    --source path/to/test/image.jpg \
    --img 640 \
    --conf-thres 0.5
```

### 3. 测试视频

```bash
python detect.py \
    --weights runs/train/traffic_lights_yolov5s/weights/best.pt \
    --source path/to/test/video.mp4 \
    --img 640 \
    --conf-thres 0.5
```

### 4. 评估指标

训练完成后，查看 `runs/train/traffic_lights_yolov5s/results.png` 中的指标：
- **mAP@0.5**: 平均精度（IoU=0.5）
- **mAP@0.5:0.95**: 平均精度（IoU=0.5到0.95）
- **Precision**: 精确率
- **Recall**: 召回率

---

## 模型导出 / Model Export

### 1. 导出为ONNX格式

```bash
python export.py \
    --weights runs/train/traffic_lights_yolov5s/weights/best.pt \
    --img 640 \
    --include onnx
```

### 2. 导出为TensorRT格式

```bash
python export.py \
    --weights runs/train/traffic_lights_yolov5s/weights/best.pt \
    --img 640 \
    --include engine \
    --device 0
```

### 3. 复制模型到项目目录

训练完成后，将最佳模型复制到项目目录：

```bash
cp runs/train/traffic_lights_yolov5s/weights/best.pt \
   src/isspa_object_detection/camera_based_detection/yolov5_ros/scripts/yolov5s_traffic_lights.pt
```

---

## 训练技巧 / Training Tips

### 1. 数据增强

YOLOv5默认启用数据增强，包括：
- 随机翻转
- 随机缩放
- 颜色抖动
- Mosaic增强
- MixUp增强

可以在 `data/hyps/hyp.scratch-med.yaml` 中调整增强参数。

### 2. 类别不平衡

如果某个类别（如黄灯）样本较少：
- 增加该类别的数据
- 使用类别权重
- 使用Focal Loss

### 3. 过拟合处理

如果出现过拟合：
- 增加数据量
- 使用数据增强
- 减少模型复杂度
- 使用Dropout
- 早停（Early Stopping）

### 4. 提高检测精度

- 使用更大的模型（yolov5m, yolov5l）
- 增加训练轮数
- 使用更大的输入尺寸（--img 1280）
- 使用更好的数据增强策略

---

## 常见问题 / FAQ

### Q1: 训练时显存不足怎么办？

- 减小 `--batch-size`
- 减小 `--img` 尺寸
- 使用更小的模型（yolov5n, yolov5s）

### Q2: 训练很慢怎么办？

- 使用GPU训练（`--device 0`）
- 增加 `--workers` 数量
- 使用多GPU训练
- 减小输入图像尺寸

### Q3: 检测精度不高怎么办？

- 增加训练数据
- 检查标注质量
- 使用更大的模型
- 调整置信度阈值
- 增加训练轮数

### Q4: 如何选择模型大小？

- **yolov5n**: 最快，适合实时检测，精度较低
- **yolov5s**: 平衡速度和精度（推荐）
- **yolov5m**: 更高精度，速度较慢
- **yolov5l/x**: 最高精度，速度最慢

---

## 参考资源 / References

- [YOLOv5官方文档](https://docs.ultralytics.com/)
- [YOLOv5 GitHub](https://github.com/ultralytics/yolov5)
- [LabelImg GitHub](https://github.com/heartexlabs/labelImg)
- [Roboflow](https://roboflow.com/)

---

## 下一步 / Next Steps

训练完成后：
1. 将模型复制到项目目录
2. 按照 `TRAFFIC_LIGHT_INTEGRATION.md` 集成到ISSPA系统
3. 在实际场景中测试和调优



