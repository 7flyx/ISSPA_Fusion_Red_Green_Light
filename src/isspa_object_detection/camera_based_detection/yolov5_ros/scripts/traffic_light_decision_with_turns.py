#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
红绿灯决策模块（支持转向灯）
Traffic Light Decision Module (with Turn Support)

该模块接收红绿灯检测结果，并根据检测状态和车辆转向意图做出决策：
- 红灯：停止车辆
- 绿灯：允许通行
- 黄灯：减速或停止
- 根据车辆转向意图（直行/左转/右转）选择对应的红绿灯

This module receives traffic light detection results and makes decisions based on 
detection state and vehicle turn intention:
- Red light: Stop vehicle
- Green light: Allow passage
- Yellow light: Slow down or stop
- Select corresponding traffic light based on turn intention (straight/left/right)
"""

import rospy
import math
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseActionGoal, GoalID
from yolov5_ros.msg import TrafficLightState

class TrafficLightDecisionWithTurns:
    def __init__(self):
        rospy.init_node('traffic_light_decision_with_turns', anonymous=False)
        
        # 参数配置
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.yellow_light_action = rospy.get_param('~yellow_light_action', 'stop')  # 'stop' or 'slow'
        self.slow_down_speed = rospy.get_param('~slow_down_speed', 0.2)  # 减速速度
        self.turn_angle_threshold = rospy.get_param('~turn_angle_threshold', 0.5)  # 转向角度阈值（弧度）
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 5.0)  # 前瞻距离（米）
        
        # 状态变量
        self.current_traffic_lights = {}  # 存储所有检测到的红绿灯 {direction: {state, confidence}}
        self.current_turn_intention = 'straight'  # 'straight', 'left', 'right'
        self.last_detection_time = rospy.Time(0)
        self.detection_timeout = rospy.Duration(2.0)  # 2秒未检测到则认为失效
        
        # 当前车辆位置和路径
        self.current_pose = None
        self.current_path = None
        
        # 订阅红绿灯检测结果
        self.sub_traffic_light = rospy.Subscriber(
            '/traffic_light_state', 
            TrafficLightState, 
            self.traffic_light_callback,
            queue_size=10
        )
        
        # 订阅导航路径（用于判断转向意图）
        self.sub_path = rospy.Subscriber(
            '/move_base/NavfnROS/plan',  # 或 '/move_base/GlobalPlanner/plan'
            Path,
            self.path_callback,
            queue_size=10
        )
        
        # 订阅当前位姿（用于计算转向）
        self.sub_pose = rospy.Subscriber(
            '/current_pose',  # 需要根据实际话题调整
            PoseStamped,
            self.pose_callback,
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
        
        # 发布当前转向意图
        self.pub_turn_intention = rospy.Publisher(
            '/traffic_light_turn_intention',
            String,
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
        
        rospy.loginfo("Traffic Light Decision Module (with turns) initialized")
        
    def traffic_light_callback(self, msg):
        """处理红绿灯检测结果"""
        if msg.confidence >= self.confidence_threshold:
            # 解析红绿灯方向（从state或frame_id中提取）
            # 假设state编码为: 0-2=直行, 3-5=左转, 6-8=右转
            direction = self._get_direction_from_state(msg.state)
            
            self.current_traffic_lights[direction] = {
                'state': msg.state,
                'confidence': msg.confidence,
                'color': self._get_color_from_state(msg.state),
                'timestamp': msg.stamp
            }
            self.last_detection_time = msg.stamp
            rospy.logdebug(f"Traffic light detected: direction={direction}, state={msg.state}, confidence={msg.confidence:.2f}")
    
    def _get_direction_from_state(self, state):
        """从状态值获取方向"""
        if 0 <= state <= 2:
            return 'straight'
        elif 3 <= state <= 5:
            return 'left'
        elif 6 <= state <= 8:
            return 'right'
        else:
            return 'unknown'
    
    def _get_color_from_state(self, state):
        """从状态值获取颜色"""
        color_map = {
            0: 'red', 3: 'red', 6: 'red',      # 红灯
            1: 'green', 4: 'green', 7: 'green', # 绿灯
            2: 'yellow', 5: 'yellow', 8: 'yellow'  # 黄灯
        }
        return color_map.get(state, 'unknown')
    
    def path_callback(self, msg):
        """处理导航路径，计算转向意图"""
        self.current_path = msg
        if len(msg.poses) < 2:
            self.current_turn_intention = 'straight'
            return
        
        # 计算路径的转向意图
        self.current_turn_intention = self._calculate_turn_intention(msg)
        
        # 发布转向意图
        turn_msg = String()
        turn_msg.data = self.current_turn_intention
        self.pub_turn_intention.publish(turn_msg)
    
    def _calculate_turn_intention(self, path):
        """
        根据路径计算转向意图
        分析路径前几个点的方向变化
        """
        if len(path.poses) < 3:
            return 'straight'
        
        # 获取路径的前几个点（在lookahead_distance范围内）
        lookahead_poses = []
        for i, pose in enumerate(path.poses):
            if i == 0:
                continue  # 跳过当前位置
            
            # 计算距离（简化，假设在map坐标系）
            if i < len(path.poses):
                lookahead_poses.append(pose)
                if len(lookahead_poses) >= 5:  # 分析前5个点
                    break
        
        if len(lookahead_poses) < 2:
            return 'straight'
        
        # 计算路径的总转向角度
        angles = []
        for i in range(len(lookahead_poses) - 1):
            p1 = lookahead_poses[i].pose.position
            p2 = lookahead_poses[i + 1].pose.position
            
            # 计算方向角
            angle = math.atan2(p2.y - p1.y, p2.x - p1.x)
            angles.append(angle)
        
        # 计算角度变化
        if len(angles) < 2:
            return 'straight'
        
        angle_change = angles[-1] - angles[0]
        
        # 归一化到[-pi, pi]
        while angle_change > math.pi:
            angle_change -= 2 * math.pi
        while angle_change < -math.pi:
            angle_change += 2 * math.pi
        
        # 判断转向
        if abs(angle_change) < self.turn_angle_threshold:
            return 'straight'
        elif angle_change > 0:
            return 'left'
        else:
            return 'right'
    
    def pose_callback(self, msg):
        """更新当前位姿"""
        self.current_pose = msg
    
    def nav_cmd_vel_callback(self, msg):
        """保存导航栈的速度命令"""
        self.nav_cmd_vel = msg
        self.nav_cmd_vel_received = True
    
    def decision_timer_callback(self, event):
        """定时器回调：根据红绿灯状态和转向意图做出决策并发布速度命令"""
        current_time = rospy.Time.now()
        
        # 检查检测结果是否超时
        if (current_time - self.last_detection_time) > self.detection_timeout:
            # 超时未检测到，恢复导航栈的原始命令
            if self.nav_cmd_vel_received:
                self.pub_cmd_vel.publish(self.nav_cmd_vel)
                self.pub_should_stop.publish(Bool(False))
            return
        
        # 根据当前转向意图选择对应的红绿灯
        relevant_traffic_light = self.current_traffic_lights.get(self.current_turn_intention)
        
        # 如果没有检测到对应方向的红绿灯，尝试使用直行红绿灯（作为fallback）
        if relevant_traffic_light is None:
            relevant_traffic_light = self.current_traffic_lights.get('straight')
        
        # 如果还是没有，使用任何检测到的红绿灯
        if relevant_traffic_light is None and len(self.current_traffic_lights) > 0:
            relevant_traffic_light = list(self.current_traffic_lights.values())[0]
        
        # 根据红绿灯状态做出决策
        decision = Int32()
        cmd_vel = Twist()
        should_stop = False
        
        if relevant_traffic_light is None:
            # 未检测到相关红绿灯，保持导航栈的原始命令
            if self.nav_cmd_vel_received:
                cmd_vel = self.nav_cmd_vel
            decision.data = 0
        else:
            color = relevant_traffic_light['color']
            state = relevant_traffic_light['state']
            decision.data = state
            
            if color == 'red':
                # 红灯：停止车辆
                cmd_vel = Twist()  # 零速度
                should_stop = True
                rospy.loginfo(f"RED LIGHT ({self.current_turn_intention}): Vehicle stopped")
                
            elif color == 'green':
                # 绿灯：允许通行，使用导航栈的速度命令
                if self.nav_cmd_vel_received:
                    cmd_vel = self.nav_cmd_vel
                should_stop = False
                rospy.logdebug(f"GREEN LIGHT ({self.current_turn_intention}): Vehicle allowed to proceed")
                
            elif color == 'yellow':
                # 黄灯：根据配置决定是停止还是减速
                if self.yellow_light_action == 'stop':
                    cmd_vel = Twist()
                    should_stop = True
                    rospy.loginfo(f"YELLOW LIGHT ({self.current_turn_intention}): Vehicle stopped")
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
                    rospy.loginfo(f"YELLOW LIGHT ({self.current_turn_intention}): Vehicle slowing down")
        
        # 发布速度命令和停止标志
        self.pub_decision.publish(decision)
        self.pub_cmd_vel.publish(cmd_vel)
        self.pub_should_stop.publish(Bool(should_stop))

if __name__ == '__main__':
    try:
        decision_node = TrafficLightDecisionWithTurns()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



