# CPU模式运行YOLOv5
# Running YOLOv5 in CPU Mode

## 为什么使用CPU模式？

1. **虚拟机环境**: 虚拟机通常无法直接访问GPU
2. **模型较小**: 红绿灯检测模型（yolov5n）在CPU上也能运行
3. **简单部署**: 不需要配置CUDA和GPU驱动

## CPU模式性能

| 模型 | 分辨率 | CPU FPS | 延迟 |
|------|--------|---------|------|
| yolov5n | 416x416 | 3-8 | 150-300ms |
| yolov5n | 320x320 | 5-12 | 80-200ms |
| yolov5s | 416x416 | 1-3 | 300-1000ms |
| yolov5s | 320x320 | 2-5 | 200-500ms |

**注意**: CPU模式比GPU慢很多，但对于红绿灯检测（不需要实时性）通常足够。

## 配置方法

### 方法1: 使用launch文件参数（推荐）

```bash
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_cpu:=true \
    use_pc_gpu:=false \
    model_size:=n \
    image_size:=416
```

### 方法2: 手动设置device参数

```bash
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    model_size:=n \
    image_size:=416
```

launch文件默认使用CPU模式。

## CPU优化建议

### 1. 使用最小的模型

```bash
# 使用yolov5n（最小最快）
model_size:=n
```

### 2. 降低输入分辨率

```bash
# 使用416x416或更小
image_size:=416
# 或
image_size:=320
```

### 3. 降低帧率

在`detect_ros.py`中，可以降低发布频率：

```python
rate = rospy.Rate(5)  # 5 Hz instead of 10 Hz
```

### 4. 使用多线程（如果CPU核心多）

PyTorch默认会使用多线程，可以通过环境变量控制：

```bash
export OMP_NUM_THREADS=4  # 使用4个线程
export MKL_NUM_THREADS=4
```

## 性能优化

### 1. 安装优化的PyTorch

```bash
# 安装CPU版本的PyTorch（通常更快）
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### 2. 使用ONNX Runtime（可选，更快）

```bash
# 安装ONNX Runtime
pip3 install onnxruntime

# 转换模型为ONNX格式（在训练后）
python3 export.py --weights yolov5n_traffic_lights.pt --include onnx
```

### 3. 降低检测频率

修改`detect_ros.py`，每N帧检测一次：

```python
frame_skip = 2  # 每2帧检测一次
frame_count = 0
```

## 测试CPU性能

```bash
# 测试检测速度
python3 << EOF
import torch
import time
from pathlib import Path

# 加载模型
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
model.eval()

# 创建测试图像
import numpy as np
test_img = np.random.randint(0, 255, (416, 416, 3), dtype=np.uint8)

# 预热
for _ in range(5):
    _ = model(test_img)

# 测试
times = []
for _ in range(20):
    start = time.time()
    _ = model(test_img)
    times.append(time.time() - start)

print(f"平均推理时间: {np.mean(times)*1000:.1f}ms")
print(f"FPS: {1/np.mean(times):.1f}")
EOF
```

## 常见问题

### 问题1: CPU推理太慢

**解决**:
- 使用yolov5n模型
- 降低分辨率到320x320
- 降低检测频率（每2-3帧检测一次）

### 问题2: 内存占用高

**解决**:
- 使用更小的模型（yolov5n）
- 降低批次大小（batch_size=1）

### 问题3: 多线程性能不佳

**解决**:
```bash
# 限制线程数
export OMP_NUM_THREADS=2
export MKL_NUM_THREADS=2
```

## 推荐配置（虚拟机）

```bash
roslaunch navigation_stack pavs_navigation_pc_only.launch \
    camera_topic:=/camera/color/image_raw \
    enable_visualization:=true \
    use_cpu:=true \
    model_size:=n \
    image_size:=320 \
    confidence_threshold:=0.5
```

这个配置在大多数CPU上应该能达到5-10 FPS，对于红绿灯检测足够。

## 与GPU模式对比

| 配置 | FPS | 延迟 | 适用场景 |
|------|-----|------|----------|
| GPU (yolov5s, 640) | 30-60 | 20-30ms | 实时检测 |
| CPU (yolov5n, 416) | 3-8 | 150-300ms | 虚拟机/无GPU |
| CPU (yolov5n, 320) | 5-12 | 80-200ms | 虚拟机/无GPU（推荐） |

## 注意事项

1. **延迟**: CPU模式延迟较高（100-300ms），但对红绿灯检测影响不大
2. **帧率**: 不要期望实时性能，3-8 FPS通常足够
3. **资源**: CPU模式会占用较多CPU资源，确保有足够的CPU核心

