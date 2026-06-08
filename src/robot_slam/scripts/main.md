# main.py - 机器人竞赛主控节点

## 概述

`main.py` 是 abot 机器人参加竞赛任务的核心控制节点。它负责串联**视觉检测点巡航** → **VLM 线索识别** → **按线索导航至任务点** → **精密停车** → **终点动作**的完整自主流程。

- 语言: Python 2
- ROS 节点名: `navigation_demo`
- 核心类: `navigation_demo`

---

## 整体任务流程

```
初始化 ROS 节点
  │
  ├─ 等待 IMU 激活
  ├─ 延时 5s 系统稳定
  ├─ 播放开始音频
  ├─ 执行起点动作 start24()
  │
  ├─ [检测阶段] 遍历检测点 points=[10,11,12,13]
  │   ├─ goto_detection_point(point)    # 导航到检测点（含预对准）
  │   ├─ call_fruit_detection_service() # 调用 VLM 识别线索
  │   ├─ tts_client()                   # 语音播报识别结果
  │   └─ 保存映射后的任务编号
  │
  ├─ [任务阶段] go_to_task_positions()
  │   └─ 对每个 task_id:
  │       ├─ goto(target, 5s)           # move_base 粗导航
  │       ├─ AutoSinglePointTest().run() # 精密停车
  │       ├─ tts_client()               # 语音播报到达
  │       └─ escape()                   # 逃逸离开挡板区
  │
  └─ [终点阶段]
      ├─ goto(goals[16], 10s)           # move_base 粗到位
      ├─ align_final_yaw()              # 航向角闭环修正
      ├─ adjust_position()              # 激光雷达贴边校准
      └─ tts_client("已到达终点")
```

---

## 全局变量

| 变量 | 类型 | 说明 |
|------|------|------|
| `VLM_TO_TASK` | dict | VLM 返回编号 → 内部任务编号映射 (31→1, 32→2, ..., 51→9) |
| `TASK_TO_VLM` | dict | 反向映射，用于语音播报时还原原始编号 |
| `time_val` | int | 终点动作计时变量 (start24/end13 共用) |
| `clue` | int | 线索计数器 (第1/2/3...条线索) |
| `points` | list | 预设检测点索引 `[10, 11, 12, 13]` |
| `task_numbers` | list | 存储识别到的任务编号 (1-9) |
| `goals` | list | 从 launch 参数解析的所有导航点位 `[[x,y,yaw], ...]` |

---

## 类 `navigation_demo` 详解

### 构造函数 `__init__()`

初始化所有 ROS 通信接口和配置参数：

#### 发布者 (Publishers)

| 话题 | 消息类型 | 用途 |
|------|----------|------|
| `/initialpose` | `PoseWithCovarianceStamped` | 设置机器人初始位姿 |
| `/voiceWords` | `String` | 播报到达消息 (预留) |
| `/cmd_vel` | `Twist` | 直接控制底盘速度 |

#### 订阅者 (Subscribers)

| 话题 | 消息类型 | 回调 | 用途 |
|------|----------|------|------|
| `/scan` | `LaserScan` | `scan_callback` | 激光雷达数据，供 `adjust_position` 贴边使用 |
| `/odom` | `Odometry` | `odom_callback` | 里程计，提取当前航向角 |
| `/start_mission` | `String` | `start_mission_callback` | 任务启动信号 (空挂) |

#### 服务客户端 (Service Clients)

| 服务名 | 类型 | 用途 |
|--------|------|------|
| `/fruit_detection` | `Trigger` | 触发 VLM 视觉识别，返回线索编号 |
| `/tts_service` | `StringService` | 语音播报文本 |

#### 动作客户端 (Action Client)

| 动作服务 | 类型 | 用途 |
|----------|------|------|
| `move_base` | `MoveBaseAction` | ROS 导航栈标准接口 |

---

### 核心导航方法

#### `goto(p, timeout=60)`
向 `move_base` 发送导航目标，等待完成或超时。返回 `True`/`False`。

- `p`: `[x, y, yaw_deg]` 目标点位姿
- `timeout`: 超时秒数
- 超时时自动取消目标，返回 `False`

#### `goto_detection_point(point)`
检测点专用导航。支持**预对准 → 锁 yaw 直行 → yaw 闭环修正**三段式流程，避免 TEB 在最后 0.6m 重新优化航向角导致停车偏差。

流程：
1. 按 `detect_prealign_mode` 方向生成预对准目标 (默认 `back`，距离默认 0.35m)
2. `goto(prealign_goal)` 到达预对准位姿
3. `locked_approach_detection_point()` 锁 yaw 直行至拍照点
4. 可选 `align_detection_yaw()` 拍照前 yaw 闭环修正

### 视觉检测方法

#### `call_fruit_detection_service()`
设置 `/detect` 参数为 1 后调用 `/fruit_detection` 服务，返回识别到的数字字符串 (如 `"31"`) 或 `"无"`。

#### `mission(point)`
单个检测点的完整流程：导航 → 识别 → 播报 → 保存。将 VLM 返回值映射为内部任务编号存入 `task_numbers`。

### 任务执行方法

#### `go_to_task_positions()`
按 `task_numbers` 顺序依次导航到各任务点并执行精密停车。关键特性：
- 导航超时默认 8s (`task_nav_timeout`)
- 到达判定：move_base 成功 **或** 距离 ≤ `task_nav_accept_dist` (默认 0.45m)
- 导航失败时自动逃逸+重试
- 每次停车后播报到达，然后逃逸离开挡板区
- 所有任务完成才允许去终点

#### `execute_mission()`
完整任务流程入口。支持 `use_fixed_task_positions` 模式跳过视觉扫描直接使用预设任务列表。

### 精密运动控制

#### `adjust_position(side_target, back_target)`
基于激光雷达的贴边校准。控制机器人使：
- 左侧 (+90°) 距离达到 `side_target`
- 后方 (-180°) 距离达到 `back_target`
- 航向角对齐 `target_yaw`

PID 闭环控制，超时 9s。

#### `align_detection_yaw(yaw_deg)`
拍照前低速闭环修正 yaw，避免 move_base 到点后最后一刻大幅旋转。超时 3s。

#### `align_final_yaw(yaw_deg)`
终点贴边前 yaw 闭环对齐，确保按正确车体方向做激光校准。

#### `locked_approach_detection_point(yaw_deg)`
从预对准点到拍照点的短距离直行段，不再交给 move_base，避免 TEB 重新优化 yaw。

### 固定动作方法

| 方法 | 动作 |
|------|------|
| `start24()` | 起点动作：前移 + 左移 (linear.x=0.25, linear.y=0.1)，持续 1.3s |
| `end13()` | 终点动作：后退 + 右移 (linear.x=-0.3, linear.y=0.3) |
| `rotate()` | 原地左转 (angular.z=1.0)，持续 0.8s |
| `right()` | 右侧平移 (linear.y=-0.5)，持续 2s |

### 辅助方法

| 方法 | 用途 |
|------|------|
| `set_pose(p)` | 发布初始位姿到 `/initialpose` |
| `stop_movement()` | 发布零速度停止运动 |
| `get_range_at_angle(angle)` | 从激光数据获取指定角度的距离 |
| `normalize_angle(angle)` | 角度归一化到 [-π, π] |
| `clamp(value, min, max)` | 数值范围限制 |
| `tts_client(text)` | 调用 TTS 服务语音播报 |
| `parse_fixed_task_ids()` | 解析 `fixed_task_ids` 参数为任务编号列表 |
| `wait_for_odom_yaw(timeout)` | 等待里程计航向角可用 |
| `distance_to_goal_xy(target)` | 计算当前位置到目标的距离 |
| `nav_reached_by_state_and_distance(nav_ok, target)` | 综合导航状态和距离判断是否到达 |

---

## ROS 参数 (Parameter Server)

所有参数均通过 launch 文件注入 (`~` 为私有参数命名空间)。

### 预对准参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~detect_prealign_enabled` | bool | `true` | 是否启用检测点预对准 |
| `~detect_prealign_mode` | str | `"back"` | 预对准方向 (back/front/left/right) |
| `~detect_prealign_distance` | float | `0.35` | 预对准偏移距离 (m) |
| `~detect_prealign_timeout` | float | `25` | 预对准导航超时 (s) |
| `~detect_yaw_align_at_prealign` | bool | `true` | 预对准后是否修正 yaw |
| `~detect_final_timeout` | float | `35` | 最终拍照点导航超时 (s) |

### 锁 yaw 靠近参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~detect_locked_final_approach` | bool | `true` | 是否用锁 yaw 直行靠近拍照点 |
| `~detect_locked_approach_speed` | float | `0.15` | 锁 yaw 靠近速度 (m/s) |
| `~detect_locked_approach_yaw_hold` | bool | `true` | 靠近时是否保持 yaw |
| `~detect_locked_approach_yaw_kp` | float | `0.8` | 锁 yaw PD 比例系数 |
| `~detect_locked_approach_max_yaw_vel` | float | `0.20` | 锁 yaw 最大角速度 |
| `~detect_locked_approach_timeout_margin` | float | `1.0` | 锁 yaw 超时冗余 (s) |

### Yaw 修正参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~detect_yaw_align_enabled` | bool | `true` | 是否启用检测点 yaw 闭环 |
| `~detect_yaw_align_at_photo` | bool | `false` | 拍照前是否再修 yaw |
| `~detect_yaw_tolerance` | float | `0.06` | yaw 容差 (rad, ~3.5°) |
| `~detect_yaw_align_timeout` | float | `3.0` | yaw 修正超时 (s) |
| `~detect_yaw_kp` | float | `1.2` | yaw PD 比例系数 |
| `~detect_yaw_min_vel` | float | `0.08` | 最小角速度 (rad/s) |
| `~detect_yaw_max_vel` | float | `0.45` | 最大角速度 (rad/s) |
| `~detect_yaw_stable_count` | int | `4` | yaw 稳定判定次数 |
| `~detect_photo_settle_time` | float | `0.25` | 拍照前稳定延时 (s) |
| `~detect_capture_wait` | float | `0.5` | 设置检测参数后等待时间 (s) |

### 固定任务点参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~use_fixed_task_positions` | bool | `false` | 跳过视觉扫描，使用预设任务列表 |
| `~fixed_task_ids` | str | `""` | 固定任务 ID 列表 (逗号/分号分隔，支持 1-9 或 31-51) |

### 终点参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~final_nav_timeout` | float | `10.0` | 终点导航超时 (s) |
| `~final_yaw_align_timeout` | float | `3.0` | 终点 yaw 修正超时 (s) |
| `~final_yaw_tolerance` | float | `0.05` | 终点 yaw 容差 (rad) |
| `~final_yaw_kp` | float | `1.2` | 终点 yaw PD 比例系数 |
| `~final_yaw_min_vel` | float | `0.08` | 终点 yaw 最小角速度 |
| `~final_yaw_max_vel` | float | `0.45` | 终点 yaw 最大角速度 |
| `~final_yaw_stable_count` | int | `3` | 终点 yaw 稳定判定次数 |

### 任务导航参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `~task_nav_timeout` | float | `8.0` | 任务点导航超时 (s) |
| `~task_nav_retry_timeout` | float | `5.0` | 任务点导航重试超时 (s) |
| `~task_nav_accept_dist` | float | `0.45` | 任务点到达距离阈值 (m) |

### Launch 文件必需参数

| 参数 | 说明 |
|------|------|
| `~goalListX` | 逗号分隔的 X 坐标列表 |
| `~goalListY` | 逗号分隔的 Y 坐标列表 |
| `~goalListYaw` | 逗号分隔的朝向角度列表 |

---

## 导航点位约定

| 索引 | 用途 |
|------|------|
| 10, 11, 12, 13 | 预设检测点 (VLM 线索识别) |
| 1-9 | 任务点 (对应 VLM 识别结果 31-51) |
| 16 | 终点 |

---

## 依赖关系

### ROS 包依赖
- `move_base` (导航栈)
- `ar_track_alvar` (AR 标记消息)
- `TTS_audio` (自定义 TTS 服务)
- `robot_slam` (精密停车 `auto_parking_pd`)

### 外部服务
- `/fruit_detection` — VLM 视觉检测 (由 `abot_vlm` 包提供)
- `/tts_service` — 文本转语音 (由 `TTS_audio` 包提供)

### 硬件依赖
- IMU (`/imu/data`)
- 激光雷达 (`/scan`)
- 里程计 (`/odom`)
- 相机 (间接通过 VLM 服务)

---

## 运行方式

```bash
# 通常通过 launch 文件启动，点位参数在 launch 中配置
roslaunch robot_slam multi_goal.launch
```

或手动运行:

```bash
rosrun robot_slam main.py \
  _goalListX:="1.0,2.0,3.0,..." \
  _goalListY:="1.0,2.0,3.0,..." \
  _goalListYaw:="0,90,180,..." \
  _use_fixed_task_positions:=false
```

---

## 注意事项

1. **Python 2 兼容**: 代码使用 Python 2 语法，TTS 调用中显式处理 unicode 编码
2. **全局变量**: `time_val`, `clue`, `task_numbers` 等使用全局变量，`execute_mission()` 中会重置
3. **IMU 等待**: 主函数启动后会阻塞等待 `/imu/data` 消息，确保 IMU 已初始化
4. **精密停车**: 任务点导航到达后委托 `AutoSinglePointTest` (来自 `auto_parking_pd.py`) 执行精密停车
5. **逃逸机制**: 每个任务点完成后执行 `escape()` 离开挡板区域，下一个任务点导航失败时也会尝试逃逸重试
6. **固定任务模式**: 设置 `use_fixed_task_positions=true` 可跳过视觉检测阶段，直接使用预设任务列表，适合调试或比赛备用
