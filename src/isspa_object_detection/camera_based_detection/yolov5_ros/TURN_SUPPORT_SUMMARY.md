# 转向红绿灯支持总结
# Turn Traffic Light Support Summary

## 问题 / Problem

在ISSPA项目中，车辆在导航过程中会遇到：
- 直行红绿灯
- 左转红绿灯  
- 右转红绿灯

需要根据车辆的转向意图选择对应的红绿灯进行决策。

## 解决方案 / Solution

提供了两种方案：

### 方案1: 简化版本（3类）- 当前默认

**特点**：
- 只区分颜色：red, green, yellow
- 通过路径分析判断转向意图
- 数据需求少，训练简单

**适用场景**：
- 简单路口（只有直行红绿灯）
- 数据量有限的情况
- 快速原型开发

**文件**：
- `data/traffic_lights.yaml`
- `traffic_light_decision.py`

### 方案2: 扩展版本（9类）- 推荐用于复杂场景

**特点**：
- 区分颜色和方向：red_straight, green_left, yellow_right 等
- 可以精确识别不同方向的红绿灯
- 决策更准确

**适用场景**：
- 复杂路口（有转向红绿灯）
- 需要高精度决策
- 数据充足的情况

**文件**：
- `data/traffic_lights_with_turns.yaml`
- `traffic_light_decision_with_turns.py`

## 快速选择 / Quick Selection

### 我应该使用哪个版本？

**使用简化版本（3类）如果**：
- ✅ 你的场景中主要是直行红绿灯
- ✅ 数据量有限（<500张图像）
- ✅ 需要快速部署

**使用扩展版本（9类）如果**：
- ✅ 你的场景中有转向红绿灯
- ✅ 数据量充足（>1000张图像）
- ✅ 需要高精度决策

## 使用步骤 / Usage Steps

### 简化版本使用

1. **训练模型**：
```bash
python train.py --data data/traffic_lights.yaml --weights yolov5s.pt --epochs 100
```

2. **启动系统**：
```bash
roslaunch yolov5_ros traffic_light_integration.launch use_turn_support:=false
```

### 扩展版本使用

1. **训练模型**：
```bash
python train.py --data data/traffic_lights_with_turns.yaml --weights yolov5s.pt --epochs 100
```

2. **启动系统**：
```bash
roslaunch yolov5_ros traffic_light_integration_with_turns.launch use_turn_support:=true
```

## 决策逻辑 / Decision Logic

### 简化版本决策流程

```
1. 检测红绿灯（red/green/yellow）
2. 分析导航路径，判断转向意图（straight/left/right）
3. 如果检测到多个红绿灯：
   - 优先选择对应转向方向的红绿灯
   - 如果没有，使用直行红绿灯
4. 根据颜色做出决策
```

### 扩展版本决策流程

```
1. 检测红绿灯（9类中的任意一类）
2. 分析导航路径，判断转向意图（straight/left/right）
3. 选择对应方向的红绿灯：
   - 左转 → 选择 left 类别
   - 右转 → 选择 right 类别
   - 直行 → 选择 straight 类别
4. 根据颜色做出决策
```

## 路径转向意图判断 / Turn Intention Detection

决策模块通过分析导航路径来判断转向意图：

1. **获取路径前几个点**（在lookahead_distance范围内）
2. **计算每个点的方向角**
3. **计算总的角度变化**
4. **判断转向**：
   - 角度变化 < threshold → 直行
   - 角度变化 > 0 → 左转
   - 角度变化 < 0 → 右转

### 参数调整

```xml
<param name="turn_angle_threshold" value="0.5"/>  <!-- 转向角度阈值（弧度） -->
<param name="lookahead_distance" value="5.0"/>   <!-- 前瞻距离（米） -->
```

## 数据标注 / Data Annotation

### 简化版本（3类）

只需标注颜色：
```
0 0.5 0.5 0.2 0.3  # red
1 0.3 0.4 0.15 0.25  # green
2 0.4 0.5 0.18 0.28  # yellow
```

### 扩展版本（9类）

需要标注颜色和方向：
```
0 0.5 0.5 0.2 0.3  # red_straight
3 0.3 0.4 0.15 0.25  # red_left
6 0.4 0.5 0.18 0.28  # red_right
1 0.5 0.5 0.2 0.3  # green_straight
4 0.3 0.4 0.15 0.25  # green_left
7 0.4 0.5 0.18 0.28  # green_right
...
```

## 升级路径 / Upgrade Path

### 从简化版本升级到扩展版本

1. **收集转向红绿灯数据**
   - 左转红绿灯图像
   - 右转红绿灯图像

2. **重新标注数据**
   - 区分直行/左转/右转
   - 使用9类标注

3. **重新训练模型**
   ```bash
   python train.py --data data/traffic_lights_with_turns.yaml ...
   ```

4. **切换到扩展版本决策模块**
   ```bash
   roslaunch yolov5_ros traffic_light_integration_with_turns.launch
   ```

## 测试建议 / Testing Recommendations

### 测试场景

1. **直行场景**
   - 车辆直行，验证是否检测直行红绿灯
   - 验证决策是否正确

2. **左转场景**
   - 车辆左转，验证是否检测左转红绿灯
   - 验证是否忽略其他方向的红绿灯

3. **右转场景**
   - 车辆右转，验证是否检测右转红绿灯
   - 验证是否忽略其他方向的红绿灯

4. **复杂场景**
   - 同时存在多个红绿灯
   - 验证是否选择正确的红绿灯

## 相关文件 / Related Files

### 配置文件
- `data/traffic_lights.yaml` - 简化版本（3类）
- `data/traffic_lights_with_turns.yaml` - 扩展版本（9类）

### 决策模块
- `traffic_light_decision.py` - 简化版本
- `traffic_light_decision_with_turns.py` - 扩展版本

### Launch文件
- `launch/traffic_light_integration.launch` - 基础版本
- `launch/traffic_light_integration_with_turns.launch` - 支持转向版本

### 文档
- `TURN_TRAFFIC_LIGHT_GUIDE.md` - 详细指南
- `TRAIN_TRAFFIC_LIGHT.md` - 训练指南

## 总结 / Summary

✅ **已实现**：
- 支持直行、左转、右转红绿灯检测
- 路径转向意图判断
- 智能红绿灯选择
- 两种方案可选（简化/扩展）

✅ **建议**：
- 简单场景：使用简化版本（3类）
- 复杂场景：使用扩展版本（9类）
- 可以逐步从简化版本升级到扩展版本



