#! /bin/bash

cmds=(
    "
    ros2 launch livox_ros_driver2 msg_MID360_launch.py 
    "
	"
    ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=RMUL \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
	"
    "
    ros2 launch rm_serial_driver rm_serial.launch.py 
    "
)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "source /home/ythare/ares/install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done