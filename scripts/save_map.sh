#!/bin/bash

# 保存二进制 .bt（默认）
ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=./map.bt

# 如果你想保存完整（非二进制压缩）的 .ot：
# ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=/home/rm/sim2real/test4_ws/map.ot -p full:=true