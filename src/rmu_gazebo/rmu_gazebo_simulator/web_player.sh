#!/bin/zsh

# Start player_web
source install/setup.sh

python3 src/rmu_gazebo_simulator/scripts/player_web/main_no_vision.py
