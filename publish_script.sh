#!/bin/bash

# ==========================================
# 陷阱机制：当按下 Ctrl+C 退出脚本时，自动清理所有后台进程
# ==========================================
trap "echo -e '\n[停止模拟] 正在清理后台进程...'; kill 0; exit" SIGINT

# 1. 先 source ROS 2 系统的环境变量
source /opt/ros/humble/setup.bash

# 2. 再 source 你当前工作空间的环境变量
source install/setup.bash

echo "开始模拟追击环境..."

# 3. 模拟比赛状态 -> 比赛进行中 (后台循环发送)
ros2 topic pub -r 10 /red_standard_robot1/game_status rm_interfaces/msg/GameStatus "{
    game_progress: 4
}" > /dev/null 2>&1 &

# 4. 模拟机器人状态 -> 满血状态 (后台循环发送)
ros2 topic pub -r 10 /red_standard_robot1/robot_status rm_interfaces/msg/RobotStatus "{
    current_hp: 400
}" > /dev/null 2>&1 &

# 5. 模拟检测到装甲板 (后台循环发送)
ros2 topic pub -r 5 /red_standard_robot1/detector/armors rm_interfaces/msg/NavigationReceive "{
  is_detect_armor: true
}" > /dev/null 2>&1 &

echo "=========================================="
echo "🎯 正在模拟敌人来回移动"
echo "轨迹: X坐标在 3.0 到 7.0 之间匀速折返"
echo "按 Ctrl + C 结束测试"
echo "=========================================="

# 6. 模拟视觉追踪的敌方目标坐标 (动态来回移动)
while true; do
    # 路线 A：从 X=3.0 匀速走向 X=7.0，步长 0.2
    for x in $(seq 3.0 0.5 4.0); do
        ros2 topic pub -1 /red_standard_robot1/tracker/target rm_interfaces/msg/Target "{
            header: {
                stamp: {sec: 0, nanosec: 0},
                frame_id: 'map'
            },
            tracking: true,
            id: '1',
            position: {x: $x, y: -3.5, z: 0.0}
        }" > /dev/null 2>&1
        sleep 0.1
    done
    
    # 路线 B：从 X=7.0 匀速走回 X=3.0，步长 -0.2
    for x in $(seq 7.0 -0.2 3.0); do
        ros2 topic pub -1 /red_standard_robot1/tracker/target rm_interfaces/msg/Target "{
            header: {
                stamp: {sec: 0, nanosec: 0},
                frame_id: 'map'
            },
            tracking: true,
            id: '1',
            position: {x: $x, y: -3.5, z: 0.0}
        }" > /dev/null 2>&1
        sleep 0.1
    done
done