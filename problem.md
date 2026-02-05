使用lidarlsam库配合mid360仿真建图
只有点云数据，没有IMU数据
位姿很漂，1是点云数据更新频率太小,2是仿真环境中特征结构较少



1. ros2 launch slam_car gazebo.launch.py 
2. ros2 launch lidarslam lidarslam.launch.py


问题：
```
[gzserver-2] [INFO] [1769584647.353998167] [LivoxPointsPlugin]: load csv file name: /home/rm/sim2real/test4_ws/install/ros2_livox_simulation/share/ros2_livox_simulation/scan_mode/mid360.csv
```

mid360 mesh文件和mid360.xacro有冲突

