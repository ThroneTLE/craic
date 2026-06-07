#!/usr/bin/env bash
### gmapping/amcl with abot ###

LOG_DIR="$HOME/craic/logs/demo"
rm -rf "$LOG_DIR"
mkdir -p "$LOG_DIR"

RUN_LOG="$HOME/craic/run_with_demo_log.sh"

gnome-terminal --window -e "bash -c '$RUN_LOG $LOG_DIR 01_roscore roscore; exec bash'" \
--tab -e "bash -c 'sleep 3; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 02_robot_with_imu roslaunch abot_bringup robot_with_imu.launch; exec bash'" \
--tab -e "bash -c 'sleep 4; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 03_navigation roslaunch robot_slam navigation.launch; exec bash'" \
--tab -e "bash -c 'sleep 5; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 04_board_localizer roslaunch robot_slam board_localizer.launch; exec bash'" \
--tab -e "bash -c 'sleep 4; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 05_usb_cam roslaunch usb_cam usb_cam-test.launch; exec bash'" \
--tab -e "bash -c 'sleep 4; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 06_vlm roslaunch abot_vlm vlm_node.launch; exec bash'" \
--tab -e "bash -c 'sleep 4; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 07_multi_goal roslaunch robot_slam multi_goal.launch; exec bash'" \
--tab -e "bash -c 'sleep 4; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 08_rviz roslaunch robot_slam view_nav.launch; exec bash'" \
--tab -e "bash -c 'sleep 4; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 09_tts rosrun TTS_audio TTS.py; exec bash'" \
--tab -e "bash -c 'sleep 4; source ~/craic/devel/setup.bash; $RUN_LOG $LOG_DIR 10_game_start roslaunch robot_slam GameStart.launch; exec bash'"
