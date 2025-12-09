# Jetson上安装ultralytics问题修复
# Fix ultralytics Installation on Jetson

## 问题描述 / Problem

安装 `ultralytics` 时，`polars` 包的依赖 `puccinialin` 无法找到，导致安装失败。

## 解决方案 / Solutions

### 方案1: 安装不包含polars的ultralytics版本（推荐）

```bash
# 安装旧版本的ultralytics（不依赖polars）
pip3 install "ultralytics<8.3.0"

# 或者指定版本
pip3 install ultralytics==8.0.196
```

### 方案2: 跳过polars依赖

```bash
# 先安装polars的依赖
pip3 install pyarrow

# 然后安装ultralytics，跳过polars
pip3 install ultralytics --no-deps
pip3 install pyyaml scipy matplotlib requests psutil torch torchvision opencv-python
```

### 方案3: 使用conda环境（如果已安装conda）

```bash
# 创建conda环境
conda create -n isspa python=3.8
conda activate isspa

# 安装ultralytics
pip install ultralytics
```

### 方案4: 不使用ultralytics（使用本地实现）

如果不需要ultralytics的完整功能，代码已经添加了fallback机制，可以不安装ultralytics。

## 推荐方案 / Recommended Solution

对于Jetson平台，推荐使用方案1：

```bash
# 安装兼容的ultralytics版本
pip3 install "ultralytics<8.3.0"

# 验证安装
python3 -c "import ultralytics; print('ultralytics installed:', ultralytics.__version__)"
```

## 如果方案1失败，尝试方案2

```bash
# 安装基础依赖
pip3 install pyyaml scipy matplotlib requests psutil opencv-python

# 安装ultralytics（跳过可选依赖）
pip3 install ultralytics --no-deps

# 手动安装必需的依赖
pip3 install pyyaml scipy matplotlib requests psutil
```

## 验证安装 / Verify Installation

```bash
python3 << EOF
try:
    import ultralytics
    print(f"✓ ultralytics installed: {ultralytics.__version__}")
except ImportError as e:
    print(f"✗ ultralytics not installed: {e}")
EOF
```

