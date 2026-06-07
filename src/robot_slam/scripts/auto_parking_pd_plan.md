# 自动泊车方案：单入口 + PD控制 + 激光精调

## 涉及文件

| 文件 | 说明 |
|------|------|
| `auto_parking_pd.py` | 泊车模块主体 |
| `main.py` | 调用 `parking.run()` + TTS后 `parking.escape()` |

## 完整流程

```
main.py
│
├─ goto(goals[point])                 检测点, timeout=60s
├─ goto(goals[task_id], timeout=5)    任务点, timeout=5s
│
├─ parking.run()
│  │
│  ├─ Phase 0: move_base 直达目标中心 (Timer+轮询, 最多 5s)
│  │   ├─ SUCCEEDED → return
│  │   ├─ dist < 0.10m + yaw_err < 0.05rad → return
│  │   ├─ 3s 净位移 < 0.2m → 抽搐, 走入口
│  │   └─ 超时 5s → 走入口
│  │
│  ├─ generate_entries() → 4入口
│  ├─ sort_entries_by_obstacle_score() → 取第1个
│  │
│  ├─ Phase 1a: move_base 到入口 (Timer+轮询, 最多 8s)
│  │   ├─ SUCCEEDED → Phase 2
│  │   ├─ dist < 0.15m → Phase 2
│  │   ├─ 3s 净位移 < 0.2m → 抽搐, Phase 1b
│  │   └─ 超时 8s → Phase 1b
│  │
│  ├─ Phase 1b: pid_goto_point(入口)  yaw 4s + translate 11s
│  │   └─ 失败 → return (Phase 2/3/4 不执行)
│  │
│  ├─ Phase 2: pid_goto_point(目标中心)  yaw 4s + translate 11s
│  │
│  ├─ Phase 3: 单挡板激光精调 (最多 4s)
│  │   └─ 取入口对面方向, 单方向 PD
│  │
│  └─ stop_robot
│
├─ TTS 语音播报
│
└─ parking.escape()  沿入口轴反向退 0.35m, 最多 5s
```

## 各 Phase 超时汇总

| Phase | 超时 | 超时行为 |
|-------|------|---------|
| main.py goto() 检测点 | 60s | 正常等待 |
<<<<<<< HEAD
| main.py goto() 任务点 | 5s | 切 PID 泊车 |
| Phase 0: move_base 直达中心 | 5s (硬编码) | 走入口 |
=======
| main.py 任务点 | 直接启动泊车 | Phase 0 内部调用 move_base |
| Phase 0: move_base 直达中心 | 10s (硬编码) | 走入口 |
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff
| Phase 1a: move_base 到入口 | 8s (硬编码) | 走 Phase 1b |
| Phase 1b-A: PID 航向对齐 | 4s | 跳过对齐, 直接平移 |
| Phase 1b-B: PID 平移 | 11s | 停在当前位置, 进 Phase 2 |
| Phase 2-A: PID 航向对齐 | 4s | 同上 |
| Phase 2-B: PID 平移 | 11s | 停在当前位置, 进 Phase 3 |
| Phase 3: 激光精调 | 4s | 停在当前位置 |
| Phase 4: 逃逸 | 5s | 能退多少算多少 |

### Phase 失败短路

```
Phase 1b 失败 → return (Phase 2/3/4 跳过)
Phase 2 失败 → Phase 3/4 继续
Phase 3 超时 → Phase 4 继续
```

## move_base 抽搐检测 (Phase 0 / Phase 1a)

```
ROS Timer 硬中断: 超时立刻 cancel_goal(), 不依赖主循环

位置历史轮询:
  每 0.1s 记录 (x, y), 保留最近 3s 历史
  3s 净位移 < 0.2m → 判定抽搐 → cancel
```

## PD 控制器

```python
class PDController:
    def update(self, error, now):
        dt = (now - last_time).to_sec()
        de = (error - last_error) / dt
        return kp * error + kd * de
```

## Phase 3: 激光精调

入口对面 = 真挡板方向, 映射到机器人局部坐标系, 单方向 PD:

```
best_entry="left" → opposite="right"
  right box角度=0° → base角度=normalize(0-target_yaw) → 归类激光方向
  单点测距, PD 调到目标值
```

## Phase 4: 逃逸

```
沿入口轴反方向退 escape_distance=0.35m
速度 escape_speed=0.10m/s
有 apply_laser_safety 保护
main.py 在 TTS 播报之后调用
```

---

## 完整可调参数

### 一、main.py 导航

| 调用 | timeout |
|------|---------|
| 检测点 `self.goto(goals[point])` | 60s |
<<<<<<< HEAD
| 任务点 `self.goto(goals[task_id], timeout=5)` | 5s |
=======
| 任务点 | 直接启动泊车, Phase 0 内部调用 move_base |
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff

### 二、Phase 0: move_base 直达目标中心

| 参数 | 值 | 说明 |
|------|-----|------|
| `enable_direct_center` | True | 开关 |
<<<<<<< HEAD
| `direct_center_timeout` | 5.0 (硬编码) | 超时 s |
=======
| `direct_center_timeout` | 10.0 (硬编码) | 超时 s |
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff
| `direct_center_tolerance` | 0.10 | dist < 此值 + yaw < 0.05 即接受 m |
| `direct_center_oscillation_window` | 3.0 (硬编码) | 抽搐窗口 s |
| `direct_center_oscillation_min_displacement` | 0.2 (硬编码) | 抽搐位移阈值 m |

### 三、入口生成与评分

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `entry_offset` | 0.34 | 入口距目标中心偏移 m |
| `enable_entry_recognition` | True | 评分开关 |
| `target_box_half_size` | 0.24 | 检测框半边长 m |
| `side_detect_width` | 0.12 | 条带宽度 m |
| `side_detect_min_points` | 4 | 条带最小点数 |
| `enable_opening_circle_detect` | True | 圆环检测开关 |
| `opening_detect_radius` | 0.30 | 圆环半径 m |
| `opening_ring_width` | 0.08 | 圆环宽度 m |
| `opening_min_clear_diff` | 3 | 最小象限差 |
| `opening_best_bonus` | 45.0 | 最佳开口奖励 |
| `opening_not_best_penalty` | 22.0 | 非最佳开口罚分 |
| `opening_count_weight` | 10.0 | 圆环点数权重 |
| `opening_unknown_penalty` | 0.0 | 无法判断开口罚分 |
| `path_corridor_width` | 0.36 | 通道宽度 m |
| `path_corridor_min_points` | 4 | 通道最小点数 |
| `path_corridor_ignore_near_start` | 0.06 | 忽略通道起点 m |
| `path_corridor_ignore_near_goal` | 0.08 | 忽略通道终点 m |
| `corridor_width` | 0.32 | 旧走廊宽度 m |
| `corridor_min_points` | 5 | 旧走廊最小点数 |
| `scan_memory_time` | 0.8 | 激光记忆时间 s |
| `recognition_max_range` | 1.6 | 采集范围 m |

### 四、Phase 1a: move_base 到入口

| 参数 | 值 | 说明 |
|------|-----|------|
| `entry_nav_timeout` | 8.0 (硬编码) | 超时 s |
| `entry_nav_tolerance` | 0.15 | dist < 此值即接受 m |

### 五、PD 控制 (Phase 1b / Phase 2)

| 参数 | 默认值 | 说明 |
|------|--------|------|
<<<<<<< HEAD
| `pid_kp_xy` | 0.6 | 位置 P 增益 |
=======
| `pid_kp_xy` | 2.4 | 位置 P 增益 |
>>>>>>> d99393dcc1bc4b13118fcda0280ed972cd35cdff
| `pid_kd_xy` | 0.2 | 位置 D 增益 |
| `pid_kp_yaw` | 1.5 | 航向 P 增益 |
| `pid_kd_yaw` | 0.3 | 航向 D 增益 |
| `pid_max_v` | 0.15 | 最大平移速度 m/s |
| `pid_max_wz` | 0.6 | 最大旋转速度 rad/s |
| `pid_yaw_align_timeout` | 4.0 | 航向对齐超时 s |
| `pid_translate_timeout` | 11.0 | 平移超时 s |
| `pos_tolerance` | 0.02 | 位置容差 m |
| `yaw_tolerance` | 0.05 | 航向容差 rad |

### 六、Phase 3: 激光精调

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `fine_tune_enabled` | True | 开关 |
| `fine_tune_timeout` | 4.0 | 超时 s |
| `fine_tune_front_back_target` | 0.24 | 前/后到挡板目标距离 m |
| `fine_tune_side_target` | 0.20 | 左/右到挡板目标距离 m |
| `fine_tune_tolerance` | 0.03 | 容差 m |
| `fine_tune_kp` | 0.3 | P 增益 |
| `fine_tune_kd` | 0.1 | D 增益 |
| `fine_tune_max_v` | 0.03 | 最大速度 m/s |

### 七、Phase 4: 逃逸

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `escape_enabled` | True | 开关 |
| `escape_distance` | 0.35 | 退出距离 m |
| `escape_speed` | 0.10 | 退出速度 m/s |
| `escape_timeout` | 5.0 | 超时 s |

### 八、激光雷达安全

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `scan_topic` | /scan_filtered | 激光话题 |
| `front_stop_dist` | 0.17 | 前/后硬停止距离 m |
| `front_slow_dist` | 0.30 | 减速距离 m |
| `front_slow_v` | 0.034 | 减速区最大速度 m/s |
| `side_stop_dist` | 0.16 | 侧方硬停止距离 m |
| `any_stop_dist` | 0.085 | 全局急停距离 m |
| `min_v` | 0.004 | 最小速度阈值 m/s |

### 九、cmd_vel 平滑

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_cmd_smoothing` | True | 开关 |
| `max_acc_x` | 1.0 | 最大加速度 m/s² |
| `max_acc_y` | 1.0 | 最大横向加速度 m/s² |
| `max_acc_wz` | 2.3 | 最大角加速度 rad/s² |

### 十、坐标系 / 目标点

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_frame` | map | |
| `base_frame` | base_footprint | |
| `target_x` | 构造参数 | 目标框中心 x m |
| `target_y` | 构造参数 | 目标框中心 y m |
| `target_yaw` | 构造参数 | 目标框朝向 度 |
