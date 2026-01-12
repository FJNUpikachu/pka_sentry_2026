colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
# colcon build --symlink-install --parallel-workers 2
# colcon build --packages-select small_gicp_relocalization 

source install/setup.sh

# ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
# world:=test_new \
# slam:=False \
# use_composition:=False \
# mapping:=False \
# use_robot_state_pub:=True \
# use_respawn:=True \
# style:=RMUL1


# ros2 launch pb2025_nav_bringup rm_sentry_reality_launch.py \
# world:=test_new \
# slam:=True \
# use_composition:=False \
# mapping:=False \
# use_robot_state_pub:=True \
# use_respawn:=True \
# style:=RMUL3

# ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
# ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py \
# world:=rmul_2024
# ros2 launch rmul24_gazebo_simulator bringup_sim.launch.py
# ros2 launch rmua19_gazebo_simulator standard_robot_a_test.launch.py
# colcon build --symlink-install --parallel-workers 1
# ros2 run rqt_tf_tree rqt_tf_tree --ros-args --remap /tf:=/red_standard_robot1/tf --remap /tf_static:=/red_standard_robot1/tf_static

# ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3

# ros2 launch rmu_gazebo_simulator bringup_sim.launch.py

cmds=(
"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
"ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py \
world:=rmuc_2025 \
slam:=False \
mapping:=False \
use_composition:=True"
# "ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=1.0 -p w:=0.3"
)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
