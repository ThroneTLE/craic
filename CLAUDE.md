# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build the entire catkin workspace (run from workspace root)
catkin_make

# Source the workspace after building
source devel/setup.bash

# Source a different workspace location (as used by some scripts)
source ~/throne_craic/devel/setup.bash
```

## Launch / Run

The full robot system is launched via `demo.sh`, which starts in gnome-terminal tabs:

```bash
# Bringup robot base (with IMU)
roslaunch abot_bringup robot_with_imu.launch

# Navigation
roslaunch robot_slam navigation.launch

# Camera driver + AR tag tracking
roslaunch track_tag usb_cam_with_calibration.launch
roslaunch track_tag ar_track_camera.launch

# VLM vision node
roslaunch abot_vlm vlm_node.launch

# Multi-goal navigation + RViz viewer
roslaunch robot_slam multi_goal.launch
roslaunch robot_slam view_nav.launch

# TTS voice output
rosrun TTS_audio TTS.py

# Game start (voice-activated)
roslaunch robot_slam GameStart.launch

# SLAM mapping
roslaunch robot_slam gmapping.launch

# Single goal navigation test
rosrun robot_slam single_goal_test.py
rosrun robot_slam entry_based_single_goal.py
```

## Individual Package Tests

```bash
# Single goal navigation test (with parameters)
rosrun robot_slam single_goal_test.py _goal_x:=1.0 _goal_y:=2.0 _goal_yaw:=0.0

# Entry-based precision navigation test
rosrun robot_slam entry_based_single_goal.py _target_x:=0.0 _target_y:=0.0

# Manual goal publisher
rosrun robot_slam pub_point.py

# OCR detection service test
rosservice call /ocr_detection "{}"

# VLM fruit detection service test
rosservice call /fruit_detection "{}"

# TTS service test
rosservice call /tts_service "data: 'Hello'"
```

## Code Architecture

This is a **ROS Catkin workspace** for an autonomous robot platform called **abot**. It integrates perception, navigation, and interaction capabilities for a robotics competition context.

### Package Layout

```
src/
├── abot_base/           # Robot hardware drivers & model
│   ├── abot_bringup/    # Serial protocol driver to base MCU, odometry, PID, IMU
│   ├── abot_imu/        # IMU sensor driver
│   ├── abot_model/      # URDF model, Gazebo simulation configs
│   └── lidar_filters/   # LiDAR scan filtering (box filter example)
├── abot_find/           # Feature-based object detection (find-object/SURF-SIFT), Qt GUI, TCP server
├── abot_vlm/            # Vision-Language Model integration (ByteDance doubao API)
├── ocr_detect/          # Chinese OCR via cnocr
├── robot_slam/          # SLAM (gmapping, hector, cartographer) + navigation (move_base/DWA/TEB)
├── track_tag/           # AR tag tracking via ar_track_alvar + PID controller
├── TTS_audio/           # Text-to-Speech via ByteDance volcano_tts WebSocket API
├── nav_command/         # Voice-based navigation commands
└── CMakeLists.txt       # Catkin workspace root
```

### ROS Communication Patterns

1. **Topics** — Sensor data, cmd_vel, status feedback
   - `/usb_cam/image_raw` — camera input for VLM, OCR, object detection
   - `/cmd_vel` — velocity commands to robot base
   - `/move_base_simple/goal` — navigation goal pose
   - `/move_base/status` — navigation state feedback
   - `/scan` / `/scan_filtered` — LiDAR data
   - `/tf` — coordinate transforms
   - `/robot_voice/tts_topic` — TTS input
   - `/shoot` — shoot command for competition mechanism
   - `/ar_pose_marker` — AR marker poses
   - `/odom` — wheel odometry

2. **Services** — Synchronous triggers for perception
   - `/fruit_detection` (Trigger) — triggers VLM fruit recognition
   - `/ocr_detection` (Trigger) — triggers OCR text recognition
   - `/llm_query` (LLMQuery) — text-only LLM query
   - `/tts_service` (StringService) — text-to-speech request

3. **ActionLib** — Long-running navigation tasks
   - `move_base` (MoveBaseAction) — navigation goals with feedback

4. **Parameter Server** — Inter-node coordination flags
   - `/detect` — triggers VLM capture (set to 1 to trigger, reset to 255)
   - `/ocr_det` — triggers OCR capture (set to 1 to trigger, reset to 255)
   - `/im_flag` — triggers image capture for calculation VLM
   - `/start` — mission start flag set by voice wake-up

### Key Robot Capabilities

- **SLAM**: gmapping for 2D grid mapping; AMCL for localization; pre-built maps stored in `robot_slam/maps/`
- **Navigation**: move_base with DWA/TEB local planners; configs in `robot_slam/params/carto/`
- **Precision Navigation**: `entry_based_single_goal.py` implements multi-entry box approach with radar-based safety; `single_goal_test.py` has fine-adjustment phase for sub-3cm accuracy
- **Multi-Goal Sequencing**: `navigate.cpp` navigates through A→B waypoint chain
- **VLM Integration**: Camera images captured via ROS parameter trigger, sent to ByteDance doubao-1-5-vision-pro-32k API for math/object recognition
- **AR Tag Tracking**: PID controller tracking AR markers for precision alignment
- **Voice Activation**: Snowboy hotword detection (`start.pmdl` model) to trigger mission start
- **TTS**: WebSocket connection to ByteDance volcano_tts API, plays via mplayer
- **Safety**: LiDAR-based obstacle detection and speed limiting in all navigation nodes

### VLM API Configuration

API keys and endpoints are in `abot_vlm/scripts/API_KEY.py` (not tracked in git, contains `YI_KEY`). Current VLM model: `doubao-1-5-vision-pro-32k-250115` via base URL `https://ark.cn-beijing.volces.com/api/v3`.

### Important Paths (hardcoded in scripts)

Several Python scripts hardcode paths under `/home/abot/throne_craic/` (separate from this workspace root `/home/abot/throne_craic/`):
- VLM temp images: `/home/abot/throne_craic/src/abot_vlm/temp/`
- OCR temp images: `/home/abot/throne_craic/src/ocr_detect/temp/`
- Voice model: `/home/abot/throne_craic/src/robot_slam/resources/models/start.pmdl`
- TTS output: `output.mp3` (in workspace root)
