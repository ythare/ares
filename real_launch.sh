#! /bin/bash

cmds=(
# mapping  
	"
    ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=RMUL_test \
    mode:=mapping  \
    lio:=pointlio \
    lio_rviz:=True \
    nav_rviz:=True
	"
# nav
	# "
    # ros2 launch rm_nav_bringup bringup_real.launch.py \
    # world:=RMUL \
    # mode:=mapping \  
    # lio:=fastlio \
    # localization:=slam_toolbox \
    # lio_rviz:=False \
    # nav_rviz:=True
	# "
    "
    ros2 launch livox_ros_driver2 msg_MID360_launch.py 
    "
    "
    ros2 launch rm_serial_driver rm_serial.launch.py 
    "
)

for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "$cmd;exec bash;"
	sleep 2
done