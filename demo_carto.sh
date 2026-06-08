### Cartographer 重定位 + abot ###
LOG_DIR=~/throne_craic/log
ARCHIVE_DIR=${LOG_DIR}/archive
ARCHIVE_KEEP=10
RUN_ID=$(date +%Y%m%d_%H%M%S)
mkdir -p "${ARCHIVE_DIR}"
if [ -s "${LOG_DIR}/log.txt" ]; then
  cp "${LOG_DIR}/log.txt" "${ARCHIVE_DIR}/log_${RUN_ID}.txt"
fi
if [ -s "${LOG_DIR}/parking_timeline.html" ]; then
  cp "${LOG_DIR}/parking_timeline.html" "${ARCHIVE_DIR}/parking_timeline_${RUN_ID}.html"
fi
: > "${LOG_DIR}/log.txt"
rm -f "${LOG_DIR}/parking_timeline.html"
ls -1t "${ARCHIVE_DIR}"/log_*.txt 2>/dev/null | tail -n +$((ARCHIVE_KEEP + 1)) | xargs -r rm -f
ls -1t "${ARCHIVE_DIR}"/parking_timeline_*.html 2>/dev/null | tail -n +$((ARCHIVE_KEEP + 1)) | xargs -r rm -f

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/throne_craic/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/throne_craic/devel/setup.bash; roslaunch robot_slam navigation.launch localization:=cartographer; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/throne_craic/devel/setup.bash; roslaunch usb_cam usb_cam-test.launch ; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/throne_craic/devel/setup.bash; roslaunch abot_vlm vlm_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/throne_craic/devel/setup.bash; roslaunch robot_slam multi_goal.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/throne_craic/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/throne_craic/devel/setup.bash; rosrun TTS_audio TTS.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/throne_craic/devel/setup.bash; roslaunch robot_slam GameStart.launch; exec bash"' \
