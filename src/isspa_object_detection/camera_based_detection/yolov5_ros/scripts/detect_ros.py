#! /usr/bin/env python3

# YOLOv5 🚀 by Ultralytics, AGPL-3.0 license
"""
Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     screen                          # screenshot
                                                     path/                           # directory
                                                     list.txt                        # list of images
                                                     list.streams                    # list of streams
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/LNwODJXcvt4'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s_openvino_model     # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""

import argparse
import csv
import os
import platform
import sys
from pathlib import Path

import torch

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# 尝试从ultralytics导入，如果失败则使用本地实现
try:
    from ultralytics.utils.plotting import Annotator, colors, save_one_box
    USE_ULTRALYTICS = True
except ImportError:
    USE_ULTRALYTICS = False
    # 如果ultralytics未安装，使用简化的本地实现
    try:
        # 尝试从utils.plots导入（但utils.plots可能也依赖ultralytics）
        from utils.plots import Annotator, colors
        # save_one_box需要单独处理
        def save_one_box(xyxy, im, file, gain=1.02, pad=10, square=False, BGR=False, save=True):
            import cv2
            xyxy = [int(x) for x in xyxy]
            b = [int(x) for x in xyxy]
            b[2] = min(b[2], im.shape[1])
            b[3] = min(b[3], im.shape[0])
            crop = im[b[1]:b[3], b[0]:b[2]]
            if save:
                cv2.imwrite(str(file), crop)
            return crop
    except ImportError:
        # 如果utils.plots也依赖ultralytics，创建简单的替代实现
        import cv2
        class Annotator:
            def __init__(self, im, line_width=None, font_size=None, font='Arial.ttf', pil=False, example=''):
                self.im = im
                self.lw = line_width or max(round(sum(im.shape) / 2 * 0.003), 2)
            def box_label(self, box, label='', color=(128, 128, 128), txt_color=(255, 255, 255)):
                p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
                cv2.rectangle(self.im, p1, p2, color, thickness=self.lw, lineType=cv2.LINE_AA)
                if label:
                    tf = max(self.lw - 1, 1)
                    w, h = cv2.getTextSize(label, 0, fontScale=self.lw / 3, thickness=tf)[0]
                    outside = p1[1] - h - 3 >= 0
                    p2 = p1[0] + w, p1[1] - h - 3 if outside else p1[1] + h + 3
                    cv2.rectangle(self.im, p1, p2, color, -1, cv2.LINE_AA)
                    cv2.putText(self.im, label, (p1[0], p1[1] - 2 if outside else p1[1] + h + 2),
                              0, self.lw / 3, txt_color, thickness=tf, lineType=cv2.LINE_AA)
            def result(self):
                return self.im
        def colors(i, bgr=False):
            palette = [(255, 56, 56), (255, 157, 151), (255, 112, 31), (255, 178, 29), (207, 210, 49),
                       (72, 249, 10), (146, 204, 23), (61, 219, 134), (26, 147, 52), (0, 212, 187)]
            c = palette[int(i) % len(palette)]
            return (c[2], c[1], c[0]) if bgr else c
        def save_one_box(xyxy, im, file, gain=1.02, pad=10, square=False, BGR=False, save=True):
            xyxy = [int(x) for x in xyxy]
            b = [int(x) for x in xyxy]
            b[2] = min(b[2], im.shape[1])
            b[3] = min(b[3], im.shape[0])
            crop = im[b[1]:b[3], b[0]:b[2]]
            if save:
                cv2.imwrite(str(file), crop)
            return crop
    rospy.logwarn("ultralytics未安装，使用本地fallback实现。建议安装: pip3 install 'ultralytics<8.3.0'")

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.torch_utils import select_device, smart_inference_mode

import rospy
# from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from yolov5_ros.msg import TrafficLightState



@smart_inference_mode()
def run(
        weights=ROOT / 'yolov5s.pt',  # model path or triton URL
        source=ROOT / 'data/images',  # file/dir/URL/glob/screen/0(webcam)
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_csv=False,  # save results in CSV format
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=True,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
):
    
    # 一个 python 文件中只能初始化一个节点
    rospy.init_node('yolov5', anonymous=False)
    image_pub = rospy.Publisher('detection_results', Image, queue_size=10)
    bbox_pub = rospy.Publisher('bounding_box', Pose2D, queue_size=10)  # 创建ROS发布者
    traffic_light_pub = rospy.Publisher('traffic_light_state', TrafficLightState, queue_size=10)  # 发布红绿灯状态
    rate = rospy.Rate(10)
    bridge = CvBridge()
    
    # 红绿灯类别名称映射（根据你的模型类别定义调整）
    # Traffic light class name mapping (adjust according to your model's class definitions)
    # 
    # 基础版本（3类）：只区分颜色
    # Basic version (3 classes): only distinguish colors
    traffic_light_classes = {
        'red': 1,
        'green': 2,
        'yellow': 3,
        'traffic_light_red': 1,
        'traffic_light_green': 2,
        'traffic_light_yellow': 3,
    }
    
    # 扩展版本（9类）：区分颜色和方向（如果使用traffic_lights_with_turns.yaml）
    # Extended version (9 classes): distinguish color and direction (if using traffic_lights_with_turns.yaml)
    # 取消下面的注释以启用扩展版本
    # Uncomment below to enable extended version
    """
    traffic_light_classes = {
        # 直行 (Straight)
        'red_straight': 1,
        'green_straight': 2,
        'yellow_straight': 3,
        # 左转 (Left turn)
        'red_left': 4,
        'green_left': 5,
        'yellow_left': 6,
        # 右转 (Right turn)
        'red_right': 7,
        'green_right': 8,
        'yellow_right': 9,
    }
    """

    ### add notes heres

    # detection_msgs = BoundingBox2D()


    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.streams') or (is_url and not is_file)
    screenshot = source.lower().startswith('screen')
    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    bs = 1  # batch_size
    if webcam:
        view_img = check_imshow(warn=True)
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
        bs = len(dataset)
    elif screenshot:
        dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())
    for path, im, im0s, vid_cap, s in dataset:
        with dt[0]:
            im = torch.from_numpy(im).to(model.device)
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize)

        # NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Define the path for the CSV file
        csv_path = save_dir / 'predictions.csv'

        # Create or append to the CSV file
        def write_to_csv(image_name, prediction, confidence):
            data = {'Image Name': image_name, 'Prediction': prediction, 'Confidence': confidence}
            with open(csv_path, mode='a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=data.keys())
                if not csv_path.is_file():
                    writer.writeheader()
                writer.writerow(data)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt
            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # 检测红绿灯状态（取置信度最高的红绿灯检测结果）
                # Detect traffic light state (take the detection result with highest confidence)
                traffic_light_state = TrafficLightState()
                traffic_light_state.state = 0  # 默认未检测到
                traffic_light_state.confidence = 0.0
                traffic_light_state.frame_id = "camera_frame"
                traffic_light_state.stamp = rospy.Time.now()
                
                max_traffic_light_conf = 0.0
                detected_traffic_light = None

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    center_x = (xyxy[0] + xyxy[2]) / 2  # 边界框的中心点x坐标
                    center_y = (xyxy[1] + xyxy[3]) / 2  # 边界框的中心点y坐标
                    rotation_angle = 0
                    label = names[c] if hide_conf else f'{names[c]}'
                    confidence = float(conf)
                    confidence_str = f'{confidence:.2f}'
                    
                    # 检查是否是红绿灯类别
                    # Check if it's a traffic light class
                    class_name = names[c].lower()
                    if class_name in traffic_light_classes:
                        if confidence > max_traffic_light_conf:
                            max_traffic_light_conf = confidence
                            detected_traffic_light = {
                                'state': traffic_light_classes[class_name],
                                'confidence': confidence
                            }

                    if save_csv:
                        write_to_csv(p.name, label, confidence_str)

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(f'{txt_path}.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))
                    if save_crop:
                        save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)

            # Stream results
            im0 = annotator.result()
            if view_img:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            ###
            try:
                scaling_factor = 0.5
                scaled_im = cv2.resize(im0, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_AREA)
                image_msg = bridge.cv2_to_imgmsg(scaled_im, encoding='bgr8')

                # 设置 ROS Image 消息的 header（可以根据实际情况调整）
                image_msg.header.stamp = rospy.Time.now()
                image_msg.header.frame_id = "camera_frame"

                # 设置 ROS Pose2D信息的 header（如果有检测结果）
                if len(det) > 0:
                    # 使用第一个检测结果的中心点
                    first_det = det[0]
                    center_x = (first_det[0] + first_det[2]) / 2
                    center_y = (first_det[1] + first_det[3]) / 2
                    pose = Pose2D()
                    pose.x = center_x
                    pose.y = center_y
                    pose.theta = 0
                    bbox_pub.publish(pose)

                # 发布到指定的 topic
                image_pub.publish(image_msg)
                
                # 发布红绿灯状态
                if detected_traffic_light is not None:
                    traffic_light_state.state = detected_traffic_light['state']
                    traffic_light_state.confidence = detected_traffic_light['confidence']
                else:
                    traffic_light_state.state = 0
                    traffic_light_state.confidence = 0.0
                traffic_light_state.stamp = rospy.Time.now()
                traffic_light_pub.publish(traffic_light_state)
                
                rate.sleep()  # 控制发布频率
            except Exception as e:
                print("*"*20)
                print(e)
                print("*"*20)
            ###


            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'image':
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                    vid_writer[i].write(im0)

        # Print time (inference-only)
        LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

    # Print results
    t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    if update:
        strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)


    # pose2d.x = 10
    # pose2d.y = 10
    # pose2d.theta = 0
    # detection_msgs.center = pose2d
    # detection_msgs.size_x = 10
    # detection_msgs.size_y = 10

    # while not rospy.is_shutdown():
    #     # Publish the message
    #     pub.publish(detection_msgs)
    #     rate.sleep()

def main():
    # 从ROS参数服务器读取参数（节点已在run函数中初始化）
    weights = rospy.get_param('~weights', str(ROOT / 'yolov5s_traffic_lights.pt'))
    source = rospy.get_param('~source', '0')
    data = rospy.get_param('~data', str(ROOT / 'data/coco128.yaml'))
    img_size = rospy.get_param('~img', 640)  # 单个值，会转换为元组
    imgsz = (img_size, img_size)
    conf_thres = rospy.get_param('~conf_thres', 0.25)
    iou_thres = rospy.get_param('~iou_thres', 0.45)
    max_det = rospy.get_param('~max_det', 1000)
    device = rospy.get_param('~device', '')
    view_img = rospy.get_param('~view_img', False)
    save_txt = rospy.get_param('~save_txt', False)
    save_csv = rospy.get_param('~save_csv', False)
    save_conf = rospy.get_param('~save_conf', False)
    save_crop = rospy.get_param('~save_crop', False)
    nosave = rospy.get_param('~nosave', True)
    classes = rospy.get_param('~classes', None)
    agnostic_nms = rospy.get_param('~agnostic_nms', False)
    augment = rospy.get_param('~augment', False)
    visualize = rospy.get_param('~visualize', False)
    update = rospy.get_param('~update', False)
    project = rospy.get_param('~project', str(ROOT / 'runs/detect'))
    name = rospy.get_param('~name', 'exp')
    exist_ok = rospy.get_param('~exist_ok', False)
    line_thickness = rospy.get_param('~line_thickness', 3)
    hide_labels = rospy.get_param('~hide_labels', False)
    hide_conf = rospy.get_param('~hide_conf', False)
    half = rospy.get_param('~half', False)
    dnn = rospy.get_param('~dnn', False)
    vid_stride = rospy.get_param('~vid_stride', 1)
    
    # 转换字符串路径为Path对象
    weights = Path(weights)
    data = Path(data) if data else None
    project = Path(project)
    
    rospy.loginfo(f"Starting YOLOv5 detection with weights: {weights}")
    rospy.loginfo(f"Source: {source}, Image size: {imgsz}, Confidence: {conf_thres}")
    
    try:
        check_requirements(ROOT / 'requirements.txt', exclude=('tensorboard', 'thop'))
        run(weights=weights, source=source, data=data, imgsz=imgsz, conf_thres=conf_thres, iou_thres=iou_thres,
            max_det=max_det, device=device, view_img=view_img, save_txt=save_txt, save_csv=save_csv,
            save_conf=save_conf, save_crop=save_crop, nosave=nosave, classes=classes, agnostic_nms=agnostic_nms,
            augment=augment, visualize=visualize, update=update, project=project, name=name, exist_ok=exist_ok,
            line_thickness=line_thickness, hide_labels=hide_labels, hide_conf=hide_conf, half=half, dnn=dnn,
            vid_stride=vid_stride)
    except Exception as e:
        rospy.logerr(f"YOLOv5 detection failed: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
        raise

if __name__ == '__main__':
    main()