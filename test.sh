#!/bin/bash

echo "🚀 Starting Mock Publishers in background..."

# 1. 模拟比赛状态: 比赛进行中 (progress = 4), 剩余时间充足
ros2 topic pub /referee/game_status pb_rm_interfaces/msg/GameStatus "{game_progress: 4, stage_remain_time: 300}" -r 10 &
PID1=$!

# 2. 模拟机器人状态: 血量健康，热量低，子弹充足 (不触发 IsAttacked)
ros2 topic pub /referee/robot_status pb_rm_interfaces/msg/RobotStatus "{current_hp: 400, shooter_17mm_1_barrel_heat: 0, projectile_allowance_17mm: 500, is_hp_deduced: false, hp_deduction_reason: 0, armor_id: 0}" -r 10 &
PID2=$!

# 3. 模拟 RFID: 探测到补给区
ros2 topic pub /referee/rfid_status pb_rm_interfaces/msg/RfidStatus "{friendly_supply_zone_non_exchange: 1}" -r 10 &
PID3=$!

# 4. 模拟视觉自瞄: 视野中没有敌人 (空数组)
ros2 topic pub /detector/armors auto_aim_interfaces/msg/Armors "{header: {frame_id: 'camera_link'}, armors: []}" -r 10 &
PID4=$!

# 5. 模拟 Tracker: 提供一个虚拟的敌方坐标给 CalculateAttackPose 使用
ros2 topic pub /tracker/target auto_aim_interfaces/msg/Target "{header: {frame_id: 'odom'}, tracking: true, id: '1', position: {x: 3.0, y: 0.0, z: 0.0}}" -r 10 &
PID5=$!

echo "✅ All mock publishers are running. Press Ctrl+C to stop."

# 捕获 Ctrl+C 信号，退出时清理后台进程
trap "echo '🛑 Stopping publishers...'; kill $PID1 $PID2 $PID3 $PID4 $PID5; exit" SIGINT

# 挂起主进程，等待子进程
wait
