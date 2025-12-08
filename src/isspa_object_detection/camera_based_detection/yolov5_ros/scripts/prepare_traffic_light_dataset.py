#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红绿灯数据集准备脚本
Traffic Light Dataset Preparation Script

该脚本帮助准备YOLOv5格式的红绿灯检测数据集
This script helps prepare traffic light detection dataset in YOLOv5 format
"""

import os
import shutil
import argparse
from pathlib import Path
import random

def create_dataset_structure(dataset_path):
    """创建数据集目录结构"""
    dataset_path = Path(dataset_path)
    
    # 创建目录结构
    dirs = [
        dataset_path / 'images' / 'train',
        dataset_path / 'images' / 'val',
        dataset_path / 'images' / 'test',
        dataset_path / 'labels' / 'train',
        dataset_path / 'labels' / 'val',
        dataset_path / 'labels' / 'test',
    ]
    
    for dir_path in dirs:
        dir_path.mkdir(parents=True, exist_ok=True)
        print(f"Created directory: {dir_path}")
    
    return dataset_path

def split_dataset(source_images_dir, source_labels_dir, dataset_path, train_ratio=0.7, val_ratio=0.2, test_ratio=0.1):
    """
    将数据集分割为训练集、验证集和测试集
    
    Args:
        source_images_dir: 源图像目录
        source_labels_dir: 源标签目录
        dataset_path: 目标数据集根目录
        train_ratio: 训练集比例
        val_ratio: 验证集比例
        test_ratio: 测试集比例
    """
    source_images_dir = Path(source_images_dir)
    source_labels_dir = Path(source_labels_dir)
    dataset_path = Path(dataset_path)
    
    # 获取所有图像文件
    image_files = list(source_images_dir.glob('*.jpg')) + \
                  list(source_images_dir.glob('*.png')) + \
                  list(source_images_dir.glob('*.jpeg'))
    
    # 随机打乱
    random.shuffle(image_files)
    
    total = len(image_files)
    train_count = int(total * train_ratio)
    val_count = int(total * val_ratio)
    
    # 分割数据集
    train_files = image_files[:train_count]
    val_files = image_files[train_count:train_count + val_count]
    test_files = image_files[train_count + val_count:]
    
    print(f"Total images: {total}")
    print(f"Train: {len(train_files)} ({len(train_files)/total*100:.1f}%)")
    print(f"Val: {len(val_files)} ({len(val_files)/total*100:.1f}%)")
    print(f"Test: {len(test_files)} ({len(test_files)/total*100:.1f}%)")
    
    # 复制文件
    def copy_files(files, split_name):
        for img_file in files:
            # 复制图像
            dst_img = dataset_path / 'images' / split_name / img_file.name
            shutil.copy2(img_file, dst_img)
            
            # 复制对应的标签文件
            label_file = source_labels_dir / (img_file.stem + '.txt')
            if label_file.exists():
                dst_label = dataset_path / 'labels' / split_name / label_file.name
                shutil.copy2(label_file, dst_label)
            else:
                print(f"Warning: Label file not found for {img_file.name}")
    
    copy_files(train_files, 'train')
    copy_files(val_files, 'val')
    copy_files(test_files, 'test')
    
    print("\nDataset preparation completed!")

def convert_annotation_format(input_file, output_file, format_from='voc', format_to='yolo'):
    """
    转换标注格式（如果需要）
    
    Args:
        input_file: 输入标注文件
        output_file: 输出标注文件
        format_from: 源格式 ('voc', 'coco', 'yolo')
        format_to: 目标格式 ('yolo')
    """
    # TODO: 实现格式转换
    # 目前假设已经是YOLO格式
    pass

def validate_yolo_labels(labels_dir):
    """
    验证YOLO格式的标签文件
    
    YOLO格式: class_id center_x center_y width height (归一化到0-1)
    """
    labels_dir = Path(labels_dir)
    label_files = list(labels_dir.glob('*.txt'))
    
    errors = []
    for label_file in label_files:
        with open(label_file, 'r') as f:
            lines = f.readlines()
            for line_num, line in enumerate(lines, 1):
                parts = line.strip().split()
                if len(parts) != 5:
                    errors.append(f"{label_file.name}:{line_num} - Invalid format (expected 5 values, got {len(parts)})")
                    continue
                
                try:
                    class_id = int(parts[0])
                    center_x = float(parts[1])
                    center_y = float(parts[2])
                    width = float(parts[3])
                    height = float(parts[4])
                    
                    # 检查值是否在有效范围内
                    if not (0 <= center_x <= 1 and 0 <= center_y <= 1 and 
                            0 <= width <= 1 and 0 <= height <= 1):
                        errors.append(f"{label_file.name}:{line_num} - Values out of range [0,1]")
                    
                    if class_id < 0 or class_id > 2:
                        errors.append(f"{label_file.name}:{line_num} - Invalid class_id (should be 0, 1, or 2)")
                        
                except ValueError as e:
                    errors.append(f"{label_file.name}:{line_num} - Invalid number format: {e}")
    
    if errors:
        print("Validation errors found:")
        for error in errors:
            print(f"  - {error}")
        return False
    else:
        print(f"All {len(label_files)} label files are valid!")
        return True

def main():
    parser = argparse.ArgumentParser(description='Prepare traffic light dataset for YOLOv5 training')
    parser.add_argument('--mode', type=str, choices=['create', 'split', 'validate'], 
                       default='create', help='Operation mode')
    parser.add_argument('--dataset_path', type=str, default='../datasets/traffic_lights',
                       help='Dataset root directory')
    parser.add_argument('--source_images', type=str, help='Source images directory (for split mode)')
    parser.add_argument('--source_labels', type=str, help='Source labels directory (for split mode)')
    parser.add_argument('--train_ratio', type=float, default=0.7, help='Training set ratio')
    parser.add_argument('--val_ratio', type=float, default=0.2, help='Validation set ratio')
    parser.add_argument('--test_ratio', type=float, default=0.1, help='Test set ratio')
    parser.add_argument('--labels_dir', type=str, help='Labels directory (for validate mode)')
    
    args = parser.parse_args()
    
    if args.mode == 'create':
        create_dataset_structure(args.dataset_path)
        print("\nDataset structure created. Now you can:")
        print("1. Place your images in images/train and images/val")
        print("2. Place your labels in labels/train and labels/val")
        print("3. Run with --mode split to automatically split your dataset")
        
    elif args.mode == 'split':
        if not args.source_images or not args.source_labels:
            print("Error: --source_images and --source_labels are required for split mode")
            return
        
        create_dataset_structure(args.dataset_path)
        split_dataset(args.source_images, args.source_labels, args.dataset_path,
                     args.train_ratio, args.val_ratio, args.test_ratio)
        
    elif args.mode == 'validate':
        if not args.labels_dir:
            print("Error: --labels_dir is required for validate mode")
            return
        validate_yolo_labels(args.labels_dir)

if __name__ == '__main__':
    main()



