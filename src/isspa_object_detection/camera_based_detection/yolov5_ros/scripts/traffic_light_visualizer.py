#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红绿灯检测可视化节点
Traffic Light Detection Visualizer

该节点订阅检测结果和决策信息，在图像上可视化显示：
- 红绿灯检测框
- 检测置信度
- 决策状态
- 转向意图和状态

This node subscribes to detection results and decision information, 
visualizing on images:
- Traffic light detection boxes
- Detection confidence
- Decision status
- Turn intention and state
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Bool
from cv_bridge import CvBridge, CvBridgeError
from yolov5_ros.msg import TrafficLightState

class TrafficLightVisualizer:
    def __init__(self):
        rospy.init_node('traffic_light_visualizer', anonymous=False)
        
        # 参数
        self.image_topic = rospy.get_param('~image_topic', '/detection_results')
        self.window_name = rospy.get_param('~window_name', 'Traffic Light Detection')
        self.show_decision = rospy.get_param('~show_decision', True)
        
        # 状态变量
        self.current_image = None
        self.traffic_light_state = None
        self.decision_state = None
        self.turn_intention = None
        self.turn_state = None
        self.should_stop = False
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 订阅话题
        self.sub_image = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1
        )
        
        self.sub_traffic_light = rospy.Subscriber(
            '/traffic_light_state',
            TrafficLightState,
            self.traffic_light_callback,
            queue_size=10
        )
        
        self.sub_decision = rospy.Subscriber(
            '/traffic_light_decision',
            Int32,
            self.decision_callback,
            queue_size=10
        )
        
        self.sub_turn_intention = rospy.Subscriber(
            '/traffic_light_turn_intention',
            String,
            self.turn_intention_callback,
            queue_size=10
        )
        
        self.sub_turn_state = rospy.Subscriber(
            '/traffic_light_turn_state',
            String,
            self.turn_state_callback,
            queue_size=10
        )
        
        self.sub_should_stop = rospy.Subscriber(
            '/traffic_light_should_stop',
            Bool,
            self.should_stop_callback,
            queue_size=10
        )
        
        # 定时器：定期更新显示
        self.timer = rospy.Timer(rospy.Duration(0.033), self.update_display)  # ~30 FPS
        
        rospy.loginfo("Traffic Light Visualizer initialized")
        rospy.loginfo(f"Displaying images from: {self.image_topic}")
        
    def image_callback(self, msg):
        """处理图像消息"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def traffic_light_callback(self, msg):
        """处理红绿灯检测结果"""
        self.traffic_light_state = msg
    
    def decision_callback(self, msg):
        """处理决策结果"""
        self.decision_state = msg.data
    
    def turn_intention_callback(self, msg):
        """处理转向意图"""
        self.turn_intention = msg.data
    
    def turn_state_callback(self, msg):
        """处理转向状态"""
        self.turn_state = msg.data
    
    def should_stop_callback(self, msg):
        """处理停止标志"""
        self.should_stop = msg.data
    
    def _get_state_text(self, state):
        """获取状态文本"""
        state_map = {
            0: "未检测到",
            1: "红灯 (RED)",
            2: "绿灯 (GREEN)",
            3: "黄灯 (YELLOW)",
            4: "左转红灯",
            5: "左转绿灯",
            6: "左转黄灯",
            7: "右转红灯",
            8: "右转绿灯",
            9: "右转黄灯"
        }
        return state_map.get(state, f"未知状态 ({state})")
    
    def _get_color(self, state):
        """根据状态获取颜色"""
        if state == 0:
            return (128, 128, 128)  # 灰色
        elif state in [1, 4, 7]:  # 红灯
            return (0, 0, 255)  # 红色
        elif state in [2, 5, 8]:  # 绿灯
            return (0, 255, 0)  # 绿色
        elif state in [3, 6, 9]:  # 黄灯
            return (0, 255, 255)  # 黄色
        else:
            return (255, 255, 255)  # 白色
    
    def update_display(self, event):
        """更新显示"""
        if self.current_image is None:
            return
        
        # 复制图像用于绘制
        display_image = self.current_image.copy()
        height, width = display_image.shape[:2]
        
        # 绘制信息面板背景
        panel_height = 150
        overlay = display_image.copy()
        cv2.rectangle(overlay, (0, 0), (width, panel_height), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, display_image, 0.3, 0, display_image)
        
        y_offset = 25
        line_height = 25
        
        # 显示红绿灯检测状态
        if self.traffic_light_state is not None:
            state_text = self._get_state_text(self.traffic_light_state.state)
            color = self._get_color(self.traffic_light_state.state)
            conf_text = f"置信度: {self.traffic_light_state.confidence:.2f}"
            
            cv2.putText(display_image, f"红绿灯状态: {state_text}", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(display_image, conf_text, 
                       (10, y_offset + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        else:
            cv2.putText(display_image, "红绿灯状态: 未检测到", 
                       (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (128, 128, 128), 2)
        
        y_offset += line_height * 2
        
        # 显示决策状态
        if self.show_decision:
            if self.decision_state is not None:
                decision_text = f"决策: {self._get_state_text(self.decision_state)}"
                decision_color = self._get_color(self.decision_state)
                cv2.putText(display_image, decision_text, 
                           (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, decision_color, 2)
            
            # 显示停止标志
            if self.should_stop:
                cv2.putText(display_image, "*** 车辆已停止 ***", 
                           (10, y_offset + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                cv2.putText(display_image, "车辆正常行驶", 
                           (10, y_offset + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            y_offset += line_height * 2
            
            # 显示转向信息
            if self.turn_intention is not None:
                turn_text = f"转向意图: {self.turn_intention.upper()}"
                cv2.putText(display_image, turn_text, 
                           (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            if self.turn_state is not None:
                state_text = f"转向状态: {self.turn_state.upper()}"
                cv2.putText(display_image, state_text, 
                           (10, y_offset + line_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # 显示时间戳
        timestamp = rospy.Time.now()
        time_text = f"时间: {timestamp.secs}.{timestamp.nsecs // 1000000:03d}"
        cv2.putText(display_image, time_text, 
                   (width - 200, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # 显示图像
        try:
            cv2.imshow(self.window_name, display_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error displaying image: {e}")
    
    def cleanup(self):
        """清理资源"""
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        visualizer = TrafficLightVisualizer()
        rospy.on_shutdown(visualizer.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

