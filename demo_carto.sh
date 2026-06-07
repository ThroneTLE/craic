### Cartographer 重定位 + abot ###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/craic/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/craic/devel/setup.bash; roslaunch robot_slam navigation.launch localization:=cartographer; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/craic/devel/setup.bash; roslaunch robot_slam board_localizer.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/craic/devel/setup.bash; roslaunch usb_cam usb_cam-test.launch ; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/craic/devel/setup.bash; roslaunch abot_vlm vlm_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/craic/devel/setup.bash; roslaunch robot_slam multi_goal.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/craic/devel/setup.bash; roslaunch robot_slam view_nav.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/craic/devel/setup.bash; rosrun TTS_audio TTS.py; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/craic/devel/setup.bash; roslaunch robot_slam GameStart.launch; exec bash"' \
