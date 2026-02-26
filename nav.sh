#!/bin/bash

# 1. 确保在工作空间根目录 (如果脚本在 src 下，cd .. 会回到 ros_ws)
# cd ..

# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
# colcon build --symlink-install --parallel-workers 2
# colcon build --packages-select small_gicp_relocalization 

# 2. 核心修改：必须 source bash 版本的环境变量，避免引发 Zsh 语法不兼容报错
source /opt/ros/humble/setup.bash
source install/setup.bash

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

# 3. 定义要同时启动的命令数组
cmds=(
"ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
"ros2 launch pb2025_nav_bringup rm_sentry_simulation_launch.py world:=rmuc_2025 slam:=False mapping:=False use_composition:=False"
# "ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=1.0 -p w:=0.3"
)

# 4. 核心修改：Docker 原生多进程启动逻辑
echo "====================================================="
echo "Starting ROS 2 Launch sequence in Docker environment..."
echo "====================================================="

# 获取数组长度
len=${#cmds[@]}

for i in "${!cmds[@]}"; do
    cmd="${cmds[$i]}"
    echo "Current CMD : $cmd"
    
    # 判断是否为最后一个命令
    if [[ $i -eq $((len - 1)) ]]; then
        # 最后一个命令放在前台运行，阻塞脚本，防止容器直接退出
        eval "$cmd"
    else
        # 前面的命令放在后台运行 (&)
        eval "$cmd" &
        
        # 核心修改：Gazebo 仿真器启动需要时间，增加 sleep 缓冲，避免 Nav2 过早启动找不到 TF 或 Map
        echo "Waiting for 5 seconds for the simulator to initialize..."
        sleep 5
    fi
done
