cmds=(
	"
    ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    lio_rviz:=False \
    nav_rviz:=True
	"
    "
    ros2 launch rm_serial_driver rm_serial_virtual.launch.py 
    "
)


for cmd in "${cmds[@]}"
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "source ./install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done