#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能红绿灯决策模块（支持分阶段转向决策）
Smart Traffic Light Decision Module (with Phased Turn Decision)

该模块实现了更智能的转向决策逻辑：
1. 检测到转向绿灯后，不是立即转向
2. 先前进到路口（检查是否到达转向点）
3. 检查转向方向是否有障碍物
4. 确认安全后执行转向

This module implements smarter turn decision logic:
1. After detecting turn green light, don't turn immediately
2. First advance to intersection (check if reached turn point)
3. Check if there are obstacles in turn direction
4. Execute turn after confirming safety
"""

import rospy
import math
import numpy as np
from std_msgs.msg import Int32, Bool, String
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalID
from yolov5_ros.msg import TrafficLightState

class SmartTrafficLightDecision:
    def __init__(self):
        rospy.init_node('smart_traffic_light_decision', anonymous=False)
        
        # 参数配置
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.yellow_light_action = rospy.get_param('~yellow_light_action', 'stop')
        self.slow_down_speed = rospy.get_param('~slow_down_speed', 0.2)
        self.turn_angle_threshold = rospy.get_param('~turn_angle_threshold', 0.5)
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 5.0)
        
        # 转向决策参数
        self.turn_point_distance = rospy.get_param('~turn_point_distance', 2.0)  # 转向点距离（米）
        self.turn_point_tolerance = rospy.get_param('~turn_point_tolerance', 0.5)  # 转向点容差（米）
        self.obstacle_check_range = rospy.get_param('~obstacle_check_range', 3.0)  # 障碍物检测范围（米）
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.5)  # 障碍物阈值（米）
        self.turn_clearance_time = rospy.get_param('~turn_clearance_time', 1.0)  # 转向安全确认时间（秒）
        
        # 状态变量
        self.current_traffic_lights = {}
        self.current_turn_intention = 'straight'
        self.last_detection_time = rospy.Time(0)
        self.detection_timeout = rospy.Duration(2.0)
        
        # 转向状态机
        self.turn_state = 'normal'  # 'normal', 'approaching', 'checking', 'turning', 'completed'
        self.turn_start_time = rospy.Time(0)
        self.turn_point = None  # 转向点位置
        self.reached_turn_point = False
        self.turn_direction = None  # 'left' or 'right'
        self.obstacle_clear = False
        self.obstacle_clear_start_time = rospy.Time(0)
        
        # 当前车辆位置和路径
        self.current_pose = None
        self.current_path = None
        self.laser_scan = None
        
        # 订阅红绿灯检测结果
        self.sub_traffic_light = rospy.Subscriber(
            '/traffic_light_state', 
            TrafficLightState, 
            self.traffic_light_callback,
            queue_size=10
        )
        
        # 订阅导航路径
        self.sub_path = rospy.Subscriber(
            '/move_base/NavfnROS/plan',
            Path,
            self.path_callback,
            queue_size=10
        )
        
        # 订阅当前位姿
        self.sub_pose = rospy.Subscriber(
            '/current_pose',
            PoseStamped,
            self.pose_callback,
            queue_size=10
        )
        
        # 订阅激光雷达数据（用于障碍物检测）
        self.sub_laser = rospy.Subscriber(
            '/scan',
            LaserScan,
            self.laser_callback,
            queue_size=10
        )
        
        # 订阅导航速度命令
        self.sub_cmd_vel_nav = rospy.Subscriber(
            '/cmd_vel_nav',
            Twist,
            self.nav_cmd_vel_callback,
            queue_size=10
        )
        
        # 发布决策结果
        self.pub_decision = rospy.Publisher('/traffic_light_decision', Int32, queue_size=10)
        self.pub_turn_intention = rospy.Publisher('/traffic_light_turn_intention', String, queue_size=10)
        self.pub_turn_state = rospy.Publisher('/traffic_light_turn_state', String, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_should_stop = rospy.Publisher('/traffic_light_should_stop', Bool, queue_size=10)
        
        # 保存导航栈的原始速度命令
        self.nav_cmd_vel = Twist()
        self.nav_cmd_vel_received = False
        
        # 定时器
        self.timer = rospy.Timer(rospy.Duration(0.1), self.decision_timer_callback)
        
        rospy.loginfo("Smart Traffic Light Decision Module initialized")
        
    def traffic_light_callback(self, msg):
        """处理红绿灯检测结果"""
        if msg.confidence >= self.confidence_threshold:
            direction = self._get_direction_from_state(msg.state)
            self.current_traffic_lights[direction] = {
                'state': msg.state,
                'confidence': msg.confidence,
                'color': self._get_color_from_state(msg.state),
                'timestamp': msg.stamp
            }
            self.last_detection_time = msg.stamp
    
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
            0: 'red', 3: 'red', 6: 'red',
            1: 'green', 4: 'green', 7: 'green',
            2: 'yellow', 5: 'yellow', 8: 'yellow'
        }
        return color_map.get(state, 'unknown')
    
    def path_callback(self, msg):
        """处理导航路径，计算转向意图和转向点"""
        self.current_path = msg
        if len(msg.poses) < 2:
            self.current_turn_intention = 'straight'
            self.turn_point = None
            return
        
        # 计算转向意图
        self.current_turn_intention = self._calculate_turn_intention(msg)
        
        # 如果是转向，计算转向点
        if self.current_turn_intention in ['left', 'right']:
            self.turn_point = self._find_turn_point(msg)
            self.turn_direction = self.current_turn_intention
        else:
            self.turn_point = None
            self.turn_direction = None
        
        # 发布转向意图
        turn_msg = String()
        turn_msg.data = self.current_turn_intention
        self.pub_turn_intention.publish(turn_msg)
    
    def _calculate_turn_intention(self, path):
        """根据路径计算转向意图"""
        if len(path.poses) < 3:
            return 'straight'
        
        lookahead_poses = []
        for i, pose in enumerate(path.poses):
            if i == 0:
                continue
            lookahead_poses.append(pose)
            if len(lookahead_poses) >= 5:
                break
        
        if len(lookahead_poses) < 2:
            return 'straight'
        
        angles = []
        for i in range(len(lookahead_poses) - 1):
            p1 = lookahead_poses[i].pose.position
            p2 = lookahead_poses[i + 1].pose.position
            angle = math.atan2(p2.y - p1.y, p2.x - p1.x)
            angles.append(angle)
        
        if len(angles) < 2:
            return 'straight'
        
        angle_change = angles[-1] - angles[0]
        while angle_change > math.pi:
            angle_change -= 2 * math.pi
        while angle_change < -math.pi:
            angle_change += 2 * math.pi
        
        if abs(angle_change) < self.turn_angle_threshold:
            return 'straight'
        elif angle_change > 0:
            return 'left'
        else:
            return 'right'
    
    def _find_turn_point(self, path):
        """在路径中找到转向点（路径开始明显转向的位置）"""
        if len(path.poses) < 3:
            return None
        
        # 找到路径中角度变化最大的点
        max_angle_change = 0
        turn_point_idx = 0
        
        for i in range(1, len(path.poses) - 1):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            p3 = path.poses[i + 1].pose.position
            
            angle1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
            angle2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
            
            angle_change = abs(angle2 - angle1)
            if angle_change > math.pi:
                angle_change = 2 * math.pi - angle_change
            
            if angle_change > max_angle_change:
                max_angle_change = angle_change
                turn_point_idx = i
        
        # 如果角度变化足够大，返回转向点
        if max_angle_change > self.turn_angle_threshold:
            return path.poses[turn_point_idx].pose.position
        return None
    
    def pose_callback(self, msg):
        """更新当前位姿"""
        self.current_pose = msg
    
    def laser_callback(self, msg):
        """处理激光雷达数据"""
        self.laser_scan = msg
    
    def _check_obstacle_in_turn_direction(self):
        """检查转向方向是否有障碍物"""
        if self.laser_scan is None or self.current_pose is None:
            return False
        
        # 获取车辆当前朝向
        orientation = self.current_pose.pose.orientation
        yaw = self._quaternion_to_yaw(orientation)
        
        # 确定要检查的角度范围（转向方向）
        if self.turn_direction == 'left':
            # 检查左侧（相对于车辆前方，左侧约90度）
            check_angle_start = yaw + math.pi / 2 - math.pi / 6  # 左侧60-120度范围
            check_angle_end = yaw + math.pi / 2 + math.pi / 6
        elif self.turn_direction == 'right':
            # 检查右侧
            check_angle_start = yaw - math.pi / 2 - math.pi / 6  # 右侧60-120度范围
            check_angle_end = yaw - math.pi / 2 + math.pi / 6
        else:
            return True  # 直行，不需要检查
        
        # 转换到激光雷达坐标系
        angle_min = self.laser_scan.angle_min
        angle_max = self.laser_scan.angle_max
        angle_increment = self.laser_scan.angle_increment
        
        # 检查指定角度范围内的障碍物
        min_distance = float('inf')
        for i, distance in enumerate(self.laser_scan.ranges):
            if distance < self.laser_scan.range_min or distance > self.laser_scan.range_max:
                continue
            
            angle = angle_min + i * angle_increment
            
            # 归一化角度到[0, 2π]
            while angle < 0:
                angle += 2 * math.pi
            while angle >= 2 * math.pi:
                angle -= 2 * math.pi
            
            # 检查是否在目标角度范围内
            check_start = check_angle_start
            check_end = check_angle_end
            while check_start < 0:
                check_start += 2 * math.pi
            while check_end < 0:
                check_end += 2 * math.pi
            
            if (check_start <= angle <= check_end) or \
               (check_start > check_end and (angle >= check_start or angle <= check_end)):
                if distance < min_distance:
                    min_distance = distance
        
        # 判断是否有障碍物
        if min_distance < self.obstacle_threshold:
            return False  # 有障碍物
        return True  # 无障碍物
    
    def _quaternion_to_yaw(self, quaternion):
        """将四元数转换为yaw角"""
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def _distance_to_point(self, point1, point2):
        """计算两点之间的距离"""
        return math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)
    
    def _check_reached_turn_point(self):
        """检查是否到达转向点"""
        if self.current_pose is None or self.turn_point is None:
            return False
        
        current_pos = self.current_pose.pose.position
        distance = self._distance_to_point(current_pos, self.turn_point)
        return distance < self.turn_point_tolerance
    
    def nav_cmd_vel_callback(self, msg):
        """保存导航栈的速度命令"""
        self.nav_cmd_vel = msg
        self.nav_cmd_vel_received = True
    
    def decision_timer_callback(self, event):
        """定时器回调：实现分阶段决策逻辑"""
        current_time = rospy.Time.now()
        
        # 检查检测结果是否超时
        if (current_time - self.last_detection_time) > self.detection_timeout:
            if self.nav_cmd_vel_received:
                self.pub_cmd_vel.publish(self.nav_cmd_vel)
                self.pub_should_stop.publish(Bool(False))
            self.turn_state = 'normal'
            return
        
        # 获取相关红绿灯
        relevant_traffic_light = self.current_traffic_lights.get(self.current_turn_intention)
        if relevant_traffic_light is None:
            relevant_traffic_light = self.current_traffic_lights.get('straight')
        if relevant_traffic_light is None and len(self.current_traffic_lights) > 0:
            relevant_traffic_light = list(self.current_traffic_lights.values())[0]
        
        # 决策变量
        decision = Int32()
        cmd_vel = Twist()
        should_stop = False
        
        if relevant_traffic_light is None:
            # 未检测到红绿灯，正常行驶
            if self.nav_cmd_vel_received:
                cmd_vel = self.nav_cmd_vel
            decision.data = 0
            self.turn_state = 'normal'
        else:
            color = relevant_traffic_light['color']
            state = relevant_traffic_light['state']
            decision.data = state
            
            # 根据红绿灯颜色和转向状态做出决策
            if color == 'red':
                # 红灯：停止
                cmd_vel = Twist()
                should_stop = True
                self.turn_state = 'normal'
                rospy.loginfo(f"RED LIGHT ({self.current_turn_intention}): Vehicle stopped")
                
            elif color == 'green':
                # 绿灯：根据转向状态决定行为
                if self.current_turn_intention in ['left', 'right']:
                    # 转向场景：分阶段处理
                    cmd_vel, should_stop = self._handle_turn_decision()
                else:
                    # 直行场景：直接通行
                    if self.nav_cmd_vel_received:
                        cmd_vel = self.nav_cmd_vel
                    should_stop = False
                    self.turn_state = 'normal'
                    rospy.logdebug(f"GREEN LIGHT (straight): Vehicle allowed to proceed")
                
            elif color == 'yellow':
                # 黄灯：根据配置决定
                if self.yellow_light_action == 'stop':
                    cmd_vel = Twist()
                    should_stop = True
                    self.turn_state = 'normal'
                    rospy.loginfo(f"YELLOW LIGHT: Vehicle stopped")
                else:
                    # 减速
                    if self.nav_cmd_vel_received:
                        cmd_vel = self.nav_cmd_vel
                        if abs(cmd_vel.linear.x) > self.slow_down_speed:
                            cmd_vel.linear.x = self.slow_down_speed if cmd_vel.linear.x > 0 else -self.slow_down_speed
                    should_stop = False
                    self.turn_state = 'normal'
        
        # 发布状态
        state_msg = String()
        state_msg.data = self.turn_state
        self.pub_turn_state.publish(state_msg)
        
        # 发布决策和速度命令
        self.pub_decision.publish(decision)
        self.pub_cmd_vel.publish(cmd_vel)
        self.pub_should_stop.publish(Bool(should_stop))
    
    def _handle_turn_decision(self):
        """处理转向决策（分阶段状态机）"""
        current_time = rospy.Time.now()
        cmd_vel = Twist()
        should_stop = False
        
        if self.turn_state == 'normal':
            # 状态1: 正常行驶，接近转向点
            if self.turn_point is not None:
                self.turn_state = 'approaching'
                self.turn_start_time = current_time
                rospy.loginfo(f"Approaching turn point for {self.turn_direction} turn")
        
        if self.turn_state == 'approaching':
            # 状态2: 接近转向点，继续前进
            if self._check_reached_turn_point():
                self.turn_state = 'checking'
                self.reached_turn_point = True
                self.obstacle_clear_start_time = current_time
                rospy.loginfo(f"Reached turn point, checking for obstacles in {self.turn_direction} direction")
            else:
                # 继续前进到转向点
                if self.nav_cmd_vel_received:
                    cmd_vel = self.nav_cmd_vel
                    # 接近转向点时减速
                    if self.current_pose is not None and self.turn_point is not None:
                        distance = self._distance_to_point(self.current_pose.pose.position, self.turn_point)
                        if distance < self.turn_point_distance:
                            # 减速
                            cmd_vel.linear.x *= 0.5
                            cmd_vel.angular.z *= 0.5
        
        if self.turn_state == 'checking':
            # 状态3: 检查转向方向是否有障碍物
            self.obstacle_clear = self._check_obstacle_in_turn_direction()
            
            if self.obstacle_clear:
                # 障碍物清除，等待一段时间确认安全
                if (current_time - self.obstacle_clear_start_time).to_sec() >= self.turn_clearance_time:
                    self.turn_state = 'turning'
                    rospy.loginfo(f"Obstacle clear, starting {self.turn_direction} turn")
                else:
                    # 等待确认，保持停止或低速
                    cmd_vel.linear.x = 0.1  # 低速前进
                    cmd_vel.angular.z = 0
                    should_stop = False
            else:
                # 有障碍物，等待
                cmd_vel = Twist()
                should_stop = True
                self.obstacle_clear_start_time = current_time  # 重置计时
                rospy.logwarn(f"Obstacle detected in {self.turn_direction} direction, waiting...")
        
        if self.turn_state == 'turning':
            # 状态4: 执行转向
            if self.nav_cmd_vel_received:
                cmd_vel = self.nav_cmd_vel
            should_stop = False
            
            # 检查是否完成转向（路径变直）
            if self.current_turn_intention == 'straight':
                self.turn_state = 'completed'
                rospy.loginfo(f"Turn completed")
        
        if self.turn_state == 'completed':
            # 状态5: 转向完成，恢复正常行驶
            if self.nav_cmd_vel_received:
                cmd_vel = self.nav_cmd_vel
            should_stop = False
            # 重置状态
            self.turn_state = 'normal'
            self.reached_turn_point = False
            self.obstacle_clear = False
        
        return cmd_vel, should_stop

if __name__ == '__main__':
    try:
        decision_node = SmartTrafficLightDecision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



