#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标注格式转换工具
Annotation Format Conversion Tool

支持从VOC XML、COCO JSON等格式转换为YOLO格式
Supports conversion from VOC XML, COCO JSON to YOLO format
"""

import json
import xml.etree.ElementTree as ET
import argparse
from pathlib import Path

def voc_xml_to_yolo(xml_file, output_file, class_mapping):
    """
    将VOC XML格式转换为YOLO格式
    
    Args:
        xml_file: VOC XML文件路径
        output_file: 输出YOLO格式文件路径
        class_mapping: 类别名称到ID的映射，例如 {'red': 0, 'green': 1, 'yellow': 2}
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    # 获取图像尺寸
    size = root.find('size')
    img_width = int(size.find('width').text)
    img_height = int(size.find('height').text)
    
    yolo_lines = []
    
    # 遍历所有目标
    for obj in root.findall('object'):
        class_name = obj.find('name').text.lower()
        
        # 跳过不在映射中的类别
        if class_name not in class_mapping:
            continue
        
        class_id = class_mapping[class_name]
        
        # 获取边界框坐标
        bbox = obj.find('bndbox')
        xmin = int(bbox.find('xmin').text)
        ymin = int(bbox.find('ymin').text)
        xmax = int(bbox.find('xmax').text)
        ymax = int(bbox.find('ymax').text)
        
        # 转换为YOLO格式（归一化）
        center_x = ((xmin + xmax) / 2.0) / img_width
        center_y = ((ymin + ymax) / 2.0) / img_height
        width = (xmax - xmin) / img_width
        height = (ymax - ymin) / img_height
        
        # 确保值在[0,1]范围内
        center_x = max(0, min(1, center_x))
        center_y = max(0, min(1, center_y))
        width = max(0, min(1, width))
        height = max(0, min(1, height))
        
        yolo_lines.append(f"{class_id} {center_x:.6f} {center_y:.6f} {width:.6f} {height:.6f}\n")
    
    # 写入文件
    with open(output_file, 'w') as f:
        f.writelines(yolo_lines)
    
    return len(yolo_lines)

def coco_json_to_yolo(json_file, images_dir, output_labels_dir, class_mapping):
    """
    将COCO JSON格式转换为YOLO格式
    
    Args:
        json_file: COCO JSON标注文件路径
        images_dir: 图像目录
        output_labels_dir: 输出标签目录
        class_mapping: 类别名称到ID的映射
    """
    with open(json_file, 'r') as f:
        coco_data = json.load(f)
    
    # 创建类别ID映射（COCO使用自己的类别ID）
    coco_id_to_name = {cat['id']: cat['name'].lower() for cat in coco_data['categories']}
    coco_id_to_yolo_id = {}
    for coco_id, name in coco_id_to_name.items():
        if name in class_mapping:
            coco_id_to_yolo_id[coco_id] = class_mapping[name]
    
    # 创建图像ID到文件名的映射
    image_id_to_filename = {img['id']: img['file_name'] for img in coco_data['images']}
    
    # 按图像分组标注
    annotations_by_image = {}
    for ann in coco_data['annotations']:
        image_id = ann['image_id']
        if image_id not in annotations_by_image:
            annotations_by_image[image_id] = []
        annotations_by_image[image_id].append(ann)
    
    # 转换每个图像的标注
    converted_count = 0
    for image_id, annotations in annotations_by_image.items():
        if image_id not in image_id_to_filename:
            continue
        
        filename = image_id_to_filename[image_id]
        image_info = next(img for img in coco_data['images'] if img['id'] == image_id)
        img_width = image_info['width']
        img_height = image_info['height']
        
        # 创建YOLO格式标注
        yolo_lines = []
        for ann in annotations:
            category_id = ann['category_id']
            if category_id not in coco_id_to_yolo_id:
                continue
            
            yolo_id = coco_id_to_yolo_id[category_id]
            
            # COCO格式: [x, y, width, height] (左上角坐标)
            bbox = ann['bbox']
            x, y, w, h = bbox
            
            # 转换为YOLO格式（中心点，归一化）
            center_x = (x + w / 2) / img_width
            center_y = (y + h / 2) / img_height
            width = w / img_width
            height = h / img_height
            
            # 确保值在[0,1]范围内
            center_x = max(0, min(1, center_x))
            center_y = max(0, min(1, center_y))
            width = max(0, min(1, width))
            height = max(0, min(1, height))
            
            yolo_lines.append(f"{yolo_id} {center_x:.6f} {center_y:.6f} {width:.6f} {height:.6f}\n")
        
        # 写入文件
        if yolo_lines:
            label_filename = Path(filename).stem + '.txt'
            output_file = Path(output_labels_dir) / label_filename
            with open(output_file, 'w') as f:
                f.writelines(yolo_lines)
            converted_count += 1
    
    return converted_count

def main():
    parser = argparse.ArgumentParser(description='Convert annotation formats to YOLO format')
    parser.add_argument('--format', type=str, choices=['voc', 'coco'], required=True,
                       help='Input annotation format')
    parser.add_argument('--input', type=str, required=True,
                       help='Input file or directory')
    parser.add_argument('--output', type=str, required=True,
                       help='Output directory for YOLO format labels')
    parser.add_argument('--images', type=str,
                       help='Images directory (required for COCO format)')
    parser.add_argument('--class-mapping', type=str, default='red:0,green:1,yellow:2',
                       help='Class name to ID mapping (format: name1:id1,name2:id2,...)')
    
    args = parser.parse_args()
    
    # 解析类别映射
    class_mapping = {}
    for item in args.class_mapping.split(','):
        name, id_str = item.split(':')
        class_mapping[name.lower()] = int(id_str)
    
    print(f"Class mapping: {class_mapping}")
    
    # 创建输出目录
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    if args.format == 'voc':
        # VOC格式转换
        input_path = Path(args.input)
        if input_path.is_file():
            # 单个文件
            xml_file = input_path
            output_file = output_dir / (xml_file.stem + '.txt')
            count = voc_xml_to_yolo(xml_file, output_file, class_mapping)
            print(f"Converted {xml_file.name}: {count} objects")
        elif input_path.is_dir():
            # 目录中的所有XML文件
            xml_files = list(input_path.glob('*.xml'))
            total_objects = 0
            for xml_file in xml_files:
                output_file = output_dir / (xml_file.stem + '.txt')
                count = voc_xml_to_yolo(xml_file, output_file, class_mapping)
                total_objects += count
                print(f"Converted {xml_file.name}: {count} objects")
            print(f"\nTotal: {len(xml_files)} files, {total_objects} objects")
        else:
            print(f"Error: Input path not found: {input_path}")
    
    elif args.format == 'coco':
        # COCO格式转换
        if not args.images:
            print("Error: --images directory is required for COCO format")
            return
        
        count = coco_json_to_yolo(args.input, args.images, output_dir, class_mapping)
        print(f"\nConverted {count} images")

if __name__ == '__main__':
    main()



