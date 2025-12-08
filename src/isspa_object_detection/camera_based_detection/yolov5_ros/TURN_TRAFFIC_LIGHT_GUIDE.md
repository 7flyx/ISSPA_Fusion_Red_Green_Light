# 转向红绿灯检测与决策指南
# Turn Traffic Light Detection and Decision Guide

## 问题说明 / Problem Description

在ISSPA自动驾驶项目中，车辆在导航过程中会遇到以下情况：
- **直行**：需要检测直行红绿灯
- **左转**：需要检测左转红绿灯
- **右转**：需要检测右转红绿灯

不同方向的红绿灯在视觉上可能不同，决策逻辑也需要根据车辆的转向意图来选择对应的红绿灯。

## 解决方案 / Solution

### 方案1: 扩展类别定义（推荐）

将红绿灯类别从3类扩展到9类，区分颜色和方向：

| 类别ID | 类别名称 | 说明 |
|--------|----------|------|
| 0 | red_straight | 直行红灯 |
| 1 | green_straight | 直行绿灯 |
| 2 | yellow_straight | 直行黄灯 |
| 3 | red_left | 左转红灯 |
| 4 | green_left | 左转绿灯 |
| 5 | yellow_left | 左转黄灯 |
| 6 | red_right | 右转红灯 |
| 7 | green_right | 右转绿灯 |
| 8 | yellow_right | 右转黄灯 |

**优点**：
- 可以精确识别不同方向的红绿灯
- 决策逻辑更准确
- 适合复杂路口场景

**缺点**：
- 需要更多标注数据
- 训练时间更长
- 模型复杂度增加

### 方案2: 简化版本（当前实现）

只区分颜色（3类），通过路径分析判断转向意图，然后选择对应的红绿灯。

**优点**：
- 数据需求少
- 训练简单
- 适合简单场景

**缺点**：
- 如果同时存在多个红绿灯，可能选择错误
- 需要准确的路径分析

## 实施步骤 / Implementation Steps

### 步骤1: 选择数据集配置

**选项A: 使用扩展版本（9类）**

```yaml
# 使用 data/traffic_lights_with_turns.yaml
python train.py --data data/traffic_lights_with_turns.yaml ...
```

**选项B: 使用简化版本（3类）**

```yaml
# 使用 data/traffic_lights.yaml
python train.py --data data/traffic_lights.yaml ...
```

### 步骤2: 数据标注

#### 如果使用扩展版本（9类）

标注时需要区分：
- 直行红绿灯：标注为 `red_straight`, `green_straight`, `yellow_straight`
- 左转红绿灯：标注为 `red_left`, `green_left`, `yellow_left`
- 右转红绿灯：标注为 `red_right`, `green_right`, `yellow_right`

#### 如果使用简化版本（3类）

只需标注颜色：`red`, `green`, `yellow`

### 步骤3: 训练模型

#### 扩展版本训练

```bash
python train.py \
    --data data/traffic_lights_with_turns.yaml \
    --weights yolov5s.pt \
    --img 640 \
    --batch-size 16 \
    --epochs 100 \
    --name traffic_lights_with_turns
```

#### 简化版本训练

```bash
python train.py \
    --data data/traffic_lights.yaml \
    --weights yolov5s.pt \
    --img 640 \
    --batch-size 16 \
    --epochs 100 \
    --name traffic_lights
```

### 步骤4: 使用决策模块

#### 扩展版本决策模块

```bash
rosrun yolov5_ros traffic_light_decision_with_turns.py
```

该模块会：
1. 接收红绿灯检测结果（9类）
2. 分析导航路径，判断转向意图
3. 根据转向意图选择对应的红绿灯
4. 做出决策

#### 简化版本决策模块

```bash
rosrun yolov5_ros traffic_light_decision.py
```

## 路径转向意图判断 / Turn Intention Detection

决策模块通过分析导航路径来判断转向意图：

```python
def _calculate_turn_intention(self, path):
    """
    根据路径计算转向意图
    分析路径前几个点的方向变化
    """
    # 1. 获取路径的前几个点
    # 2. 计算每个点的方向角
    # 3. 计算总的角度变化
    # 4. 判断：直行/左转/右转
```

### 参数调整

在launch文件中可以调整：

```xml
<param name="turn_angle_threshold" value="0.5"/>  <!-- 转向角度阈值（弧度） -->
<param name="lookahead_distance" value="5.0"/>     <!-- 前瞻距离（米） -->
```

## 决策逻辑 / Decision Logic

### 扩展版本决策逻辑

```
1. 检测到红绿灯（9类中的任意一类）
2. 分析导航路径，判断转向意图（straight/left/right）
3. 选择对应方向的红绿灯：
   - 如果转向意图是"左转"，选择 left 类别的红绿灯
   - 如果转向意图是"右转"，选择 right 类别的红绿灯
   - 如果转向意图是"直行"，选择 straight 类别的红绿灯
4. 根据选择的红绿灯颜色做出决策：
   - 红灯：停止
   - 绿灯：通行
   - 黄灯：减速或停止
```

### 简化版本决策逻辑

```
1. 检测到红绿灯（3类：red/green/yellow）
2. 分析导航路径，判断转向意图
3. 如果检测到多个红绿灯，优先选择：
   - 对应转向方向的红绿灯
   - 如果没有，使用直行红绿灯
   - 如果都没有，使用任何检测到的红绿灯
4. 根据选择的红绿灯颜色做出决策
```

## 使用建议 / Recommendations

### 场景1: 简单路口（只有直行红绿灯）

**推荐**：使用简化版本（3类）
- 数据需求少
- 训练简单
- 决策逻辑清晰

### 场景2: 复杂路口（有转向红绿灯）

**推荐**：使用扩展版本（9类）
- 可以精确识别不同方向的红绿灯
- 避免误判
- 决策更准确

### 场景3: 逐步升级

**推荐**：先使用简化版本，再升级到扩展版本
1. 先用3类训练基础模型
2. 收集转向红绿灯数据
3. 扩展到9类重新训练

## 数据收集建议 / Data Collection Recommendations

### 转向红绿灯数据特点

1. **左转红绿灯**：
   - 通常位于路口左侧
   - 可能有箭头指示
   - 形状可能与直行红绿灯不同

2. **右转红绿灯**：
   - 通常位于路口右侧
   - 可能有箭头指示
   - 形状可能与直行红绿灯不同

3. **直行红绿灯**：
   - 通常位于路口中央
   - 圆形或方形
   - 最常见

### 数据分布建议

- 直行红绿灯：40-50%
- 左转红绿灯：25-30%
- 右转红绿灯：25-30%

## 测试与验证 / Testing and Validation

### 测试场景

1. **直行场景**：
   - 车辆直行，检测直行红绿灯
   - 验证决策是否正确

2. **左转场景**：
   - 车辆左转，检测左转红绿灯
   - 验证是否忽略直行/右转红绿灯

3. **右转场景**：
   - 车辆右转，检测右转红绿灯
   - 验证是否忽略直行/左转红绿灯

4. **复杂场景**：
   - 同时存在多个红绿灯
   - 验证是否选择正确的红绿灯

### 验证指标

- 转向意图识别准确率
- 红绿灯选择准确率
- 决策正确率

## 常见问题 / FAQ

### Q1: 如何判断应该使用哪个版本？

**A**: 
- 如果你的场景中只有直行红绿灯，使用简化版本
- 如果有转向红绿灯，使用扩展版本
- 如果不确定，先使用简化版本，再根据实际情况升级

### Q2: 路径分析不准确怎么办？

**A**: 
- 调整 `turn_angle_threshold` 参数
- 增加 `lookahead_distance` 来提前判断
- 检查导航路径的质量

### Q3: 同时检测到多个红绿灯怎么办？

**A**: 
- 扩展版本：根据转向意图选择对应的红绿灯
- 简化版本：优先选择对应方向的红绿灯，如果没有则使用直行红绿灯

### Q4: 如何提高转向意图识别准确率？

**A**: 
- 使用更长的前瞻距离
- 平滑路径数据
- 结合车辆当前朝向

## 相关文件 / Related Files

- `data/traffic_lights.yaml` - 简化版本数据集配置（3类）
- `data/traffic_lights_with_turns.yaml` - 扩展版本数据集配置（9类）
- `traffic_light_decision.py` - 简化版本决策模块
- `traffic_light_decision_with_turns.py` - 扩展版本决策模块
- `TRAIN_TRAFFIC_LIGHT.md` - 训练指南



