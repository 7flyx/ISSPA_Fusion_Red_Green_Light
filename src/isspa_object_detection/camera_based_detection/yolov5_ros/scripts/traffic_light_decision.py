#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红绿灯决策模块
Traffic Light Decision Module

该模块接收红绿灯检测结果，并根据检测状态做出决策：
- 红灯：停止车辆
- 绿灯：允许通行
- 黄灯：减速或停止
- 未检测到：保持当前状态

This module receives traffic light detection results and makes decisions:
- Red light: Stop vehicle
- Green light: Allow passage
- Yellow light: Slow down or stop
- Not detected: Maintain current state
"""

import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal, GoalID
from yolov5_ros.msg import TrafficLightState

class TrafficLightDecision:
    def __init__(self):
        rospy.init_node('traffic_light_decision', anonymous=False)
        
        # 参数配置
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.yellow_light_action = rospy.get_param('~yellow_light_action', 'stop')  # 'stop' or 'slow'
        self.slow_down_speed = rospy.get_param('~slow_down_speed', 0.2)  # 减速速度
        
        # 状态变量
        self.current_state = 0  # 0: 未检测到, 1: 红灯, 2: 绿灯, 3: 黄灯
        self.last_detection_time = rospy.Time(0)
        self.detection_timeout = rospy.Duration(2.0)  # 2秒未检测到则认为失效
        
        # 订阅红绿灯检测结果
        self.sub_traffic_light = rospy.Subscriber(
            '/traffic_light_state', 
            TrafficLightState, 
            self.traffic_light_callback,
            queue_size=10
        )
        
        # 订阅导航速度命令（用于在绿灯时恢复）
        self.sub_cmd_vel_nav = rospy.Subscriber(
            '/cmd_vel_nav',  # 导航栈发布的原始速度命令
            Twist,
            self.nav_cmd_vel_callback,
            queue_size=10
        )
        
        # 发布决策结果（供其他模块使用）
        self.pub_decision = rospy.Publisher(
            '/traffic_light_decision', 
            Int32, 
            queue_size=10
        )
        
        # 发布最终的速度命令（融合了红绿灯决策）
        self.pub_cmd_vel = rospy.Publisher(
            '/cmd_vel', 
            Twist, 
            queue_size=10
        )
        
        # 发布是否应该停止的标志
        self.pub_should_stop = rospy.Publisher(
            '/traffic_light_should_stop',
            Bool,
            queue_size=10
        )
        
        # 保存导航栈的原始速度命令
        self.nav_cmd_vel = Twist()
        self.nav_cmd_vel_received = False
        
        # 定时器：定期检查检测结果并发布决策
        self.timer = rospy.Timer(rospy.Duration(0.1), self.decision_timer_callback)
        
        rospy.loginfo("Traffic Light Decision Module initialized")
        
    def traffic_light_callback(self, msg):
        """处理红绿灯检测结果"""
        if msg.confidence >= self.confidence_threshold:
            self.current_state = msg.state
            self.last_detection_time = msg.stamp
            rospy.logdebug(f"Traffic light detected: state={msg.state}, confidence={msg.confidence:.2f}")
        else:
            rospy.logdebug(f"Traffic light detected but confidence too low: {msg.confidence:.2f}")
    
    def nav_cmd_vel_callback(self, msg):
        """保存导航栈的速度命令"""
        self.nav_cmd_vel = msg
        self.nav_cmd_vel_received = True
    
    def decision_timer_callback(self, event):
        """定时器回调：根据红绿灯状态做出决策并发布速度命令"""
        current_time = rospy.Time.now()
        
        # 检查检测结果是否超时
        if (current_time - self.last_detection_time) > self.detection_timeout:
            # 超时未检测到，恢复导航栈的原始命令
            if self.nav_cmd_vel_received:
                self.pub_cmd_vel.publish(self.nav_cmd_vel)
                self.pub_should_stop.publish(Bool(False))
            return
        
        # 根据红绿灯状态做出决策
        decision = Int32()
        decision.data = self.current_state
        self.pub_decision.publish(decision)
        
        cmd_vel = Twist()
        should_stop = False
        
        if self.current_state == 1:  # 红灯
            # 停止车辆
            cmd_vel = Twist()  # 零速度
            should_stop = True
            rospy.loginfo("RED LIGHT: Vehicle stopped")
            
        elif self.current_state == 2:  # 绿灯
            # 允许通行，使用导航栈的速度命令
            if self.nav_cmd_vel_received:
                cmd_vel = self.nav_cmd_vel
            should_stop = False
            rospy.logdebug("GREEN LIGHT: Vehicle allowed to proceed")
            
        elif self.current_state == 3:  # 黄灯
            # 根据配置决定是停止还是减速
            if self.yellow_light_action == 'stop':
                cmd_vel = Twist()
                should_stop = True
                rospy.loginfo("YELLOW LIGHT: Vehicle stopped")
            else:  # slow
                # 减速
                if self.nav_cmd_vel_received:
                    cmd_vel = self.nav_cmd_vel
                    # 限制最大速度
                    if abs(cmd_vel.linear.x) > self.slow_down_speed:
                        cmd_vel.linear.x = self.slow_down_speed if cmd_vel.linear.x > 0 else -self.slow_down_speed
                    if abs(cmd_vel.angular.z) > 0.2:
                        cmd_vel.angular.z = 0.2 if cmd_vel.angular.z > 0 else -0.2
                should_stop = False
                rospy.loginfo("YELLOW LIGHT: Vehicle slowing down")
        else:  # 未检测到 (state == 0)
            # 保持导航栈的原始命令
            if self.nav_cmd_vel_received:
                cmd_vel = self.nav_cmd_vel
            should_stop = False
        
        # 发布速度命令和停止标志
        self.pub_cmd_vel.publish(cmd_vel)
        self.pub_should_stop.publish(Bool(should_stop))

if __name__ == '__main__':
    try:
        decision_node = TrafficLightDecision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



