#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Jetson平台YOLOv5优化配置
Jetson Platform YOLOv5 Optimization Configuration

该脚本提供Jetson平台特定的优化配置
This script provides Jetson platform-specific optimization configurations
"""

import torch
import os

def get_jetson_config():
    """
    获取Jetson优化配置
    
    Returns:
        dict: 优化配置字典
    """
    config = {
        # 设备配置
        'device': '0' if torch.cuda.is_available() else 'cpu',
        
        # 使用FP16精度（减少显存占用）
        'half': True,
        
        # 输入图像尺寸（降低分辨率以提高速度）
        'imgsz': 416,  # 而不是640
        
        # 批次大小（Jetson显存有限）
        'batch_size': 1,
        
        # 数据加载线程数（减少CPU占用）
        'workers': 2,
        
        # 模型选择（使用最小的模型）
        'model': 'yolov5n.pt',  # 或 'yolov5s.pt'
        
        # 置信度阈值
        'conf_thres': 0.5,
        
        # NMS阈值
        'iou_thres': 0.45,
        
        # 最大检测数量
        'max_det': 1000,
    }
    
    return config

def optimize_for_jetson():
    """
    应用Jetson优化设置
    """
    # 设置CUDA设备
    if torch.cuda.is_available():
        torch.cuda.set_device(0)
        
        # 启用TensorFloat-32 (TF32) 加速（如果支持）
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
        
        # 优化CUDA内存分配
        os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128'
        
        print("✓ CUDA优化已启用")
    else:
        print("⚠ CUDA不可用，使用CPU模式")

def check_jetson_resources():
    """
    检查Jetson资源使用情况
    """
    if torch.cuda.is_available():
        print(f"GPU设备: {torch.cuda.get_device_name(0)}")
        print(f"显存总量: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.2f} GB")
        print(f"当前显存使用: {torch.cuda.memory_allocated(0) / 1024**3:.2f} GB")
        print(f"显存缓存: {torch.cuda.memory_reserved(0) / 1024**3:.2f} GB")
    else:
        print("CUDA不可用")

if __name__ == '__main__':
    print("Jetson优化配置")
    print("=" * 50)
    
    # 检查资源
    check_jetson_resources()
    
    # 应用优化
    optimize_for_jetson()
    
    # 获取配置
    config = get_jetson_config()
    print("\n推荐配置:")
    for key, value in config.items():
        print(f"  {key}: {value}")

