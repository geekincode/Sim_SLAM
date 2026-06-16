# Slam Car 键盘控制说明

本项目实现了通过键盘控制slam_car机器人运动的功能，支持在Gazebo仿真环境中使用WSAD或类似键位控制机器人移动。

## 功能说明

- I: 前进
- , (逗号): 后退
- J: 左转
- L: 右转
- U: 左前移动
- O: 右前移动
- N: 左后移动
- M: 右后移动
- Q: 退出程序

## 使用方法

### 1. 编译项目

```bash
cd /home/rm/sim2real/test4_ws
colcon build --packages-select slam_car
source install/setup.bash
```

### 2. 启动Gazebo仿真

```bash
# 启动Gazebo仿真环境
ros2 launch slam_car gazebo.launch.py
```

### 3. 启动控制器

在另一个终端中：

```bash
# 启动机器人控制器
ros2 launch slam_car control.launch.py
```

### 4. 运行键盘控制节点

在另一个终端中：

```bash
# 运行键盘控制节点
ros2 run slam_car keyboard_control
```

然后按照终端上的提示使用键盘控制机器人运动。

## 参数配置

键盘控制节点有两个可配置参数：

- `linear_speed`: 线速度，默认值为0.5 m/s
- `angular_speed`: 角速度，默认值为0.5 rad/s

要使用自定义参数启动节点：

```bash
ros2 run slam_car keyboard_control --ros-args -p linear_speed:=0.8 -p angular_speed:=1.0
```

## 注意事项

1. 请确保在运行键盘控制节点的终端中有焦点（即能够接收键盘输入）
2. 控制仅在按下按键时有效，松开按键后机器人会停止
3. 为了获得最佳体验，建议在运行键盘控制节点的终端中保持焦点
4. 如果机器人移动过快或过慢，可以调整linear_speed和angular_speed参数

## 系统架构

该项目采用ROS 2的发布/订阅模式，键盘控制节点向`/cmd_vel`话题发布[geometry_msgs::Twist](file:///home/rm/sim2real/test4_ws/src/ros2_msg_conversions/geometry_msgs/include/geometry_msgs/msg/twist.hpp#L48-L65)消息，
由差动驱动控制器接收并转换为轮子的实际运动。


```
ros2 topic pub /model/slam_car/cmd_vel_4wd geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.2}}'
```


```
ros2 topic pub /model/slam_car/cmd_wheel_vel geometry_msgs/msg/Twist '{linear: {x: 15.0, y: -15.0}, angular: {x: 15.0, y: -15.0}}'
```