# craic

ROS Catkin 机器人系统工作空间，面向移动机器人竞赛/任务平台，整合底盘驱动、SLAM 建图、导航定位、AR 标签跟踪、OCR/VLM 感知、语音唤醒和 TTS 播报等功能。

这个仓库更像一个“完整机器人系统集成工程”，重点不是单个算法 demo，而是把多个 ROS 包串成可以执行任务的工作流。

## 项目定位

- **系统类型**：ROS1 / Catkin 工作空间
- **核心能力**：建图、定位、导航、目标识别、语音交互、任务执行
- **主要语言**：C++、Python、Shell
- **适用场景**：移动机器人综合任务、室内导航、视觉/语音交互、比赛系统调试

## 目录结构

```text
src/
├── abot_base/       # 底盘、IMU、URDF/Gazebo 模型和雷达过滤
├── abot_find/       # find-object 2D/3D 特征目标检测与 TCP 服务
├── abot_rpp/        # Regulated Pure Pursuit 局部规划器插件
├── abot_vlm/        # 视觉语言模型接口与视觉识别服务
├── nav_command/     # 自定义导航命令消息
├── ocr_detect/      # OCR 识别节点
├── robot_slam/      # SLAM、定位、导航、多点任务和语音唤醒
├── track_tag/       # AR 标签跟踪与相机标定
└── TTS_audio/       # 文本转语音服务
```

## 主要功能

| 模块 | 功能 |
| :--- | :--- |
| SLAM / 定位 | 支持 gmapping、Cartographer、AMCL 等建图与定位流程，地图文件保存在 `robot_slam/maps/`。 |
| 导航 | 基于 `move_base` 完成目标点导航、多目标序列导航和局部路径规划。 |
| 精准对位 | 使用 AR 标签、雷达/里程计反馈和 PID 调整完成接近目标点后的精细控制。 |
| 视觉感知 | 包含 OCR、find-object、VLM 图像识别等感知节点，可通过服务或参数触发。 |
| 语音交互 | 支持语音唤醒、TTS 播报和任务开始信号触发。 |
| 机器人底盘 | 包含底盘 bringup、IMU、URDF/Gazebo 模型和雷达过滤配置。 |

## 构建方式

在工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

如果脚本中使用了固定路径，需要根据本机工作空间位置修改相关路径。部分历史脚本默认路径为 `/home/abot/demo/`。

## 常用启动入口

完整系统可参考：

```bash
bash demo.sh
```

常用单模块启动：

```bash
# 导航
roslaunch robot_slam navigation.launch

# gmapping 建图
roslaunch robot_slam gmapping.launch

# Cartographer 建图
bash carto_slam.sh

# AR 标签相机
roslaunch track_tag usb_cam_with_calibration.launch
roslaunch track_tag ar_track_camera.launch

# VLM 视觉节点
roslaunch abot_vlm vlm_node.launch

# TTS 服务
rosrun TTS_audio TTS.py
```

## 调试命令

```bash
# 单目标导航测试
rosrun robot_slam single_goal_test.py _goal_x:=1.0 _goal_y:=2.0 _goal_yaw:=0.0

# 手动发布目标点
rosrun robot_slam pub_point.py

# OCR 服务测试
rosservice call /ocr_detection "{}"

# VLM 检测服务测试
rosservice call /fruit_detection "{}"

# TTS 服务测试
rosservice call /tts_service "data: 'Hello'"
```

## 注意事项

- 本仓库包含 `build/`、`devel/` 等历史构建产物，重新部署时建议先清理后再构建。
- VLM / TTS 等接口需要本地 API 配置，公开仓库中不应提交真实密钥。
- 机器人相关路径、串口、相机编号和雷达参数通常需要按实际硬件修改。
- 推荐把真实比赛配置、密钥和机器特定路径放在本地配置文件中，避免直接写入公开仓库。

## 技术关键词

`ROS` / `Catkin` / `SLAM` / `move_base` / `Cartographer` / `gmapping` / `AMCL` / `AR Tag` / `OCR` / `VLM` / `TTS` / `C++` / `Python`
