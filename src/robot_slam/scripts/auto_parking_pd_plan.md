# 新自动泊车方案：单入口 + PD控制

## Context

当前泊车代码问题：尝试3个入口耗时长、入框控制复杂（挡板检测/窄通道/渐进减速/死锁处理层层堆叠）、成功判定逻辑分支太多容易误判。

用户要求：选择最优入口只试一次，用简洁的PD控制直线运动到目标，超时就接受当前位置。

## 确认的设计参数

| 参数 | 值 | 说明 |
|------|-----|------|
| yaw_tolerance | 0.05 rad (~2.9°) | 航向对齐容差 |
| pos_tolerance | 0.05 m | 位置到达容差 |
| 每阶段超时 | 6 s | 超时停在当前位置继续 |
| 入口数量 | 1 个 | 只选得分最高的 |
| 控制方式 | PD (无I) | 位置x/y + 航向yaw 三个独立PD |
| 激光安全 | 需要 | 前/后/侧全向保护 |
| 运动方向 | 全向 | 根据x,y误差自动决定前进/后退/左右 |

## 新增文件

`/home/abot/craic/src/robot_slam/scripts/auto_parking_pd.py`

类名保持 `AutoSinglePointTest`，接口不变：
```python
parking = AutoSinglePointTest(target_x=..., target_y=..., target_yaw_deg=...)
parking.run()
```

main.py 需要改 import 路径（或新文件用相同类名、main.py 改从新文件 import）。

## 核心流程

```
run()
│
├─ Phase 0: move_base 直达目标中心 (轮询, 最多 5s)
│   ├─ 每隔 0.3s 检查一次状态
│   ├─ SUCCEEDED → 泊车成功, return
│   ├─ 距离目标 < 0.10m → 够近了, cancel move_base, 泊车成功, return
│   ├─ 机器人原地抽搐(距离不再缩小) → cancel, 走入口泊车
│   └─ 超时 5s → cancel, 走入口泊车
│
├─ generate_entries() → 4个入口
├─ sort_entries_by_obstacle_score() → 评分排序
├─ 取第1个（得分最低）
│
└─ 执行单入口泊车:
    │
    ├─ Phase 1a: move_base 导航到入口 (轮询, 最多 5s)
    │   ├─ SUCCEEDED → 进入 Phase 2
    │   ├─ 距离入口 < 0.15m → 够近, 进入 Phase 2
    │   ├─ 抽搐/超时 → cancel, 进入 Phase 1b
    │   └─ 离入口太远(>1.5m) → 放弃此入口, return
    │
    ├─ Phase 1b: pid_goto_point(入口坐标, 入口yaw)
    │   ├─ 子阶段A: pid_align_yaw(入口yaw) — 纯旋转, 最多 4s
    │   └─ 子阶段B: pid_translate(入口x, 入口y) + pid_yaw锁航向, 最多 6s
    │
    ├─ Phase 2: pid_goto_point(目标中心, 目标yaw)
    │   ├─ 子阶段A: pid_align_yaw(目标yaw), 最多 4s
    │   └─ 子阶段B: pid_translate(目标x, 目标y) + pid_yaw锁航向, 最多 6s
    │
    └─ 成功/失败 → stop_robot → return (main.py 继续播报)
```

### 各 Phase 超时汇总

| Phase | 超时 | 超时行为 |
|-------|------|---------|
| Phase 0: move_base 直达中心 | 5s | 走入口泊车 |
| Phase 1a: move_base 到入口 | 5s | 走 Phase 1b PID |
| Phase 1b-A: PID 航向对齐 | 4s | 跳过对齐,直接平移 |
| Phase 1b-B: PID 平移 | 6s | 停在当前位置, return |
| Phase 2-A: PID 航向对齐 | 4s | 跳过对齐,直接平移 |
| Phase 2-B: PID 平移 | 6s | 停在当前位置, 进 Phase 3 |
| Phase 3: 激光精调 | 4s | 停在当前位置, return |

**最长耗时: 5 + 4 + 6 + 4 + 6 + 4 = 29s (全部超时), 最短: Phase 0 成功 ~2s**

### Phase 0 抽搐检测

```
每 0.3s 记录机器人 map 坐标 (x,y)
维护最近 5s 的位置历史
如果 5s 内机器人净位移 (欧式距离) < 0.3m → 判定抽搐 → cancel move_base
```

## Phase 3: 激光挡板精调 (新增)

Phase 2 结束后，机器人朝向为 `target_yaw`，以此为基准做 box→base 坐标映射，用激光固定角度单点测距做最后精调。

### 原理

```
车长方体: 前/后到挡板距离统一, 左/右到挡板距离统一

  front_back_target = 0.30m  (±0.03m)   ← 一个参数管前后
  side_target        = 0.20m  (±0.03m)   ← 一个参数管左右
```

### 激光固定角度

不扫扇区，直接取激光 scan 数组的 4 个固定角度索引：

| 方向 | 角度 | 说明 |
|------|------|------|
| front | 0° | 正前方 |
| back | 180°(-180°) | 正后方 |
| left | 90° | 正左方 |
| right | -90° | 正右方 |

读到 inf/NaN → 该方向没挡板，跳过不修正。

### box 系 → base 系映射

`evaluate_target_sides` 返回 box 系 blocked 的边。Phase 2 后机器人朝向 = `target_yaw`，做映射：

```
box边     box方向     机器人朝向  →  激光方向
──────────────────────────────────────────
right    +x (0°)     θ           normalize(0° - θ)
up       +y (90°)    θ           normalize(90° - θ)
left     -x (±180°)  θ           normalize(±180° - θ)
down     -y (-90°)   θ           normalize(-90° - θ)
```

取结果中最接近 0°/90°/180°/-90° 的作为对应激光方向，单点测距。

### PD 微调逻辑

```
对每个 blocked 边:
  读激光距离 → error = 实际 - 目标
  
  mapped_to_front/back → PD 控制 linear.x
  mapped_to_left/right → PD 控制 linear.y
  
  容差 ±0.03m, 稳定 3 帧 → 该方向满足, 停调
```

### 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `fine_tune_enabled` | True | Phase 3 开关 |
| `fine_tune_timeout` | 4.0 | 超时 s |
| `fine_tune_front_back_target` | 0.30 | 前/后到挡板目标距离 m |
| `fine_tune_side_target` | 0.20 | 左/右到挡板目标距离 m |
| `fine_tune_tolerance` | 0.03 | 容差 m |
| `fine_tune_kp` | 0.3 | 微调PD-P增益 |
| `fine_tune_kd` | 0.1 | 微调PD-D增益 |
| `fine_tune_max_v` | 0.03 | 微调最大速度 m/s |

## 更新后的完整流程

```
run()
│
├─ Phase 0: move_base 直达目标中心 (轮询+抽搐检测, 最多 5s)
│   └─ 成功 → return / 失败 → 继续
│
├─ generate_entries() → 4入口
├─ sort_entries_by_obstacle_score() → 取第1个
│
├─ Phase 1a: move_base 到入口 (轮询, 最多 5s)
├─ Phase 1b: pid_goto_point(入口) (最多 10s)
│
├─ Phase 2: pid_goto_point(目标中心) (最多 10s)
│
└─ Phase 3: 激光挡板精调 (新增, 最多 4s)
    │
    ├─ evaluate_target_sides() → blocked 边列表
    ├─ 对每个 blocked 边 → box→base 映射 → 激光单点测距
    ├─ error = 激光距离 - 目标距离(0.30/0.20)
    └─ PD 微调 x/y, 容差 ±0.03m, 超时 4s → return
```

## PD控制器设计

```python
class PDController:
    """单轴PD控制器"""
    def __init__(self, kp, kd):
        self.kp = kp; self.kd = kd
        self.last_error = 0.0; self.last_time = None
    
    def update(self, error, now):
        dt = (now - self.last_time).to_sec() if self.last_time else 0.05
        if dt <= 0: dt = 0.05
        de = (error - self.last_error) / dt
        self.last_error = error; self.last_time = now
        return self.kp * error + self.kd * de
    
    def reset(self):
        self.last_error = 0.0; self.last_time = None
```

## pid_goto_point 核心逻辑

```python
def pid_goto_point(self, tx, ty, target_yaw, timeout=6.0):
    # 子阶段A: 纯旋转对齐yaw
    pd_yaw = PDController(kp_yaw, kd_yaw)
    while 未超时:
        yaw_err = normalize(target_yaw - current_yaw)
        if abs(yaw_err) < 0.05 稳定3帧: break
        wz = clamp(pd_yaw.update(yaw_err), ±max_wz)
        publish(cmd(angular.z=wz))
    
    # 子阶段B: PD直线运动 + 航向锁
    pd_x = PDController(kp_xy, kd_xy)
    pd_y = PDController(kp_xy, kd_xy)
    pd_yaw2 = PDController(kp_yaw, kd_yaw)
    
    while 未超时:
        ex = tx - rx; ey = ty - ry
        dist = sqrt(ex² + ey²)
        if dist < 0.05 稳定3帧: return True
        
        vx_map = clamp(pd_x.update(ex), ±max_v)
        vy_map = clamp(pd_y.update(ey), ±max_v)
        wz     = clamp(pd_yaw2.update(yaw_err), ±max_wz)
        
        cmd = map_to_base(vx_map, vy_map, ryaw)
        cmd.angular.z = wz
        cmd = apply_laser_safety(cmd)
        publish(cmd)

    return False  # 超时
```

## 完整可调参数列表

### 一、Phase 0: move_base 直达目标中心

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `enable_direct_center` | True | - | Phase 0 开关，false 则跳过直达 |
| `direct_center_timeout` | 5.0 | s | move_base 直达目标中心最大等待时间 |
| `direct_center_tolerance` | 0.10 | m | 距离目标中心 < 此值即接受，不等 move_base 收敛 |
| `direct_center_oscillation_window` | 5.0 | s | 抽搐检测时间窗口 |
| `direct_center_oscillation_min_displacement` | 0.3 | m | 窗口内机器人净位移 < 此值 → 判定抽搐 |

### 二、入口生成与评分

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `entry_offset` | 0.34 | m | 入口点距目标中心的偏移距离 |
| `enable_entry_recognition` | True | - | 入口智能评分开关，false 则按距离排序 |
| `target_box_half_size` | 0.24 | m | 目标框半边尺寸（检测框 48cm×48cm） |
| `side_detect_width` | 0.12 | m | 四条边外侧检测条带宽度 |
| `side_detect_min_points` | 4 | 个 | 条带内激光点数 ≥ 此值 → 该边 blocked |
| `enable_opening_circle_detect` | True | - | 圆环开口检测开关 |
| `opening_detect_radius` | 0.30 | m | 开口检测圆环半径 |
| `opening_ring_width` | 0.08 | m | 圆环宽度 |
| `opening_min_clear_diff` | 3 | 个 | 最多/最少象限点数差 ≥ 此值 → 判定 confident |
| `opening_best_bonus` | 45.0 | - | 最佳开口入口得分奖励（负值=减分=更好） |
| `opening_not_best_penalty` | 22.0 | - | 非最佳开口入口罚分 |
| `opening_count_weight` | 10.0 | - | 圆环点数权重 |
| `opening_unknown_penalty` | 0.0 | - | 无法判断开口时的罚分 |
| `path_corridor_width` | 0.36 | m | 入口→目标通道宽度 |
| `path_corridor_min_points` | 4 | 个 | 通道内激光点数 ≥ 此值 → 通道 blocked |
| `path_corridor_ignore_near_start` | 0.06 | m | 忽略通道起点附近区域 |
| `path_corridor_ignore_near_goal` | 0.08 | m | 忽略通道终点附近区域 |
| `corridor_width` | 0.32 | m | 旧版走廊检测宽度（辅助评分用） |
| `corridor_min_points` | 5 | 个 | 旧版走廊点数阈值 |
| `scan_memory_time` | 0.8 | s | 激光点云记忆时间（累积多帧） |
| `recognition_max_range` | 1.6 | m | 只采集目标点周围此范围内的激光点 |

### 三、Phase 1a: move_base 到入口

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `entry_nav_timeout` | 5.0 | s | move_base 到入口最大等待时间 |
| `entry_nav_tolerance` | 0.15 | m | 距离入口 < 此值即接受 |

### 四、PD 控制 (Phase 1b / Phase 2)

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `pid_kp_xy` | 0.6 | - | 位置平移 P 增益 |
| `pid_kd_xy` | 0.2 | - | 位置平移 D 增益 |
| `pid_kp_yaw` | 1.5 | - | 航向 P 增益 |
| `pid_kd_yaw` | 0.3 | - | 航向 D 增益 |
| `pid_max_v` | 0.15 | m/s | 平移最大速度 |
| `pid_max_wz` | 0.6 | rad/s | 旋转最大角速度 |
| `pid_yaw_align_timeout` | 4.0 | s | 航向对齐子阶段超时 |
| `pid_translate_timeout` | 6.0 | s | 平移子阶段超时 |
| `pos_tolerance` | 0.05 | m | 位置到达判定容差 |
| `yaw_tolerance` | 0.05 | rad | 航向对齐判定容差 (~2.9°) |

### 五、Phase 3: 激光挡板精调

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `fine_tune_enabled` | True | - | Phase 3 开关 |
| `fine_tune_timeout` | 4.0 | s | 精调最大耗时 |
| `fine_tune_front_back_target` | 0.30 | m | 前/后方到挡板目标距离（统一值） |
| `fine_tune_side_target` | 0.20 | m | 左/右侧到挡板目标距离（统一值） |
| `fine_tune_tolerance` | 0.03 | m | 距离容差 |
| `fine_tune_kp` | 0.3 | - | 精调 PD-P 增益 |
| `fine_tune_kd` | 0.1 | - | 精调 PD-D 增益 |
| `fine_tune_max_v` | 0.03 | m/s | 精调最大速度 |

### 六、激光雷达安全

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `scan_topic` | /scan_filtered | - | 激光雷达话题名 |
| `front_stop_dist` | 0.17 | m | 前方硬停止距离（后退时对应后方） |
| `front_slow_dist` | 0.30 | m | 前方减速距离 |
| `front_slow_v` | 0.034 | m/s | 前方减速区最大速度 |
| `side_stop_dist` | 0.16 | m | 侧方硬停止距离 |
| `any_stop_dist` | 0.085 | m | 全局紧急停止距离 |
| `min_v` | 0.004 | m/s | 最小速度阈值，低于此值视为零 |

### 七、cmd_vel 平滑

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `enable_cmd_smoothing` | True | - | 加速度平滑开关 |
| `max_acc_x` | 1.0 | m/s² | 前进/后退最大加速度 |
| `max_acc_y` | 1.0 | m/s² | 横向最大加速度 |
| `max_acc_wz` | 2.3 | rad/s² | 旋转最大角加速度 |

### 八、坐标系

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_frame` | map | map 坐标系 frame_id |
| `base_frame` | base_footprint | 机器人底盘 frame_id |

### 九、目标点（构造参数传入，也可从参数服务器读取）

| 参数 | 默认值 | 单位 | 说明 |
|------|--------|------|------|
| `target_x` | 0.0 | m | 目标框中心 x (map 系) |
| `target_y` | 0.0 | m | 目标框中心 y (map 系) |
| `target_yaw` | 0.0 | ° | 目标框朝向 (度，非弧度) |

---

**总计: 58 个可调参数**

## 复用现有代码

从 `auto_single_point_test.py` 直接复用（不重写）：
- `generate_entries()` — 4入口生成
- `sort_entries_by_obstacle_score()` + 子函数 — 评分排序
- `try_direct_goal_center()` + `send_move_base_pose()` — 直达
- `apply_laser_safety()` — 激光安全（已有前后保护）
- `lookup_robot_pose()` / `map_velocity_to_base_cmd()` / `publish_cmd()` / `stop_robot()` / `get_sector_min_range()` / `clamp()` / `normalize_angle()` — 工具函数

## main.py 需改一行

```python
# 第26行, 从:
from auto_single_point_test import AutoSinglePointTest
# 改为:
from auto_parking_pd import AutoSinglePointTest
```

## 验证方式

1. `python2 -c "import ast; ast.parse(open('auto_parking_pd.py').read()); print('OK')"`
2. 运行 `bash demo_carto.sh`，观察日志中入口选择、PD控制输出、成功/超时判定

## 未覆盖的边缘情况

- 激光安全急停后PD会继续输出速度 → 可能"顶住不动直到超时"。可接受（超时后接受当前位置）
- 入口点在机器人后方 → PD xy误差自然产生负的base_link速度 → 后退。后方激光保护生效
- 直达中心成功 → 直接return，不走入口逻辑
