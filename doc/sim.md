# Gazebo Sim 下 Livox 点云缺失问题解决方案

## 问题背景
在使用 **ros-humble-ros-gz（Gazebo Sim）** 时，机器人模型启动正常，但 **没有 `/livox_mid360/points` 点云数据**。终端提示 Livox 插件缺少 `IgnitionPluginHook`，说明插件与 Gazebo Sim 不兼容。

## 解决思路
1. **替换传感器实现**：Livox 仿真插件是 Gazebo Classic 插件，无法在 Gazebo Sim 运行。应改为 Gazebo Sim 原生 `gpu_lidar` 传感器。
2. **桥接消息到 ROS 2**：Gazebo Sim 传感器发布的是 Gazebo Transport 消息，需要用 `ros_gz_bridge` 转换为 ROS 2 `PointCloud2`。
3. **启用 Sensors 系统插件**：Gazebo Sim 只有在 world 启用 `Sensors` 系统插件时才会产生传感器数据。

## 具体实施方案

### 1、 替换 Livox 传感器定义为 `gpu_lidar`
文件：`src/livox_laser_simulation_RO2/urdf/mid360.xacro`

关键点：
- 删除 `libros2_livox.so` 插件
- 改为 `<sensor type="gpu_lidar">`
- 设置水平 / 垂直采样与视场
- 设置话题 `<topic>livox_mid360</topic>`

示意结构（节选）：
```xml
<sensor type="gpu_lidar" name="${name}">
  <topic>${topic}</topic>
  <update_rate>100</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>1024</samples>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>64</samples>
        <min_angle>-0.126</min_angle>
        <max_angle>0.964</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>200.0</max>
    </range>
  </lidar>
</sensor>
```

### 2、 启动 `ros_gz_bridge` 做点云桥接
文件：`src/slam_car/launch/gazebo.launch.py`

桥接命令示例（已在 launch 中配置）：
```
/livox_mid360@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
/livox_mid360/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```

### 3、 在 world 文件中启用 Sensors 系统插件
文件：`src/slam_car/world/living_room/living_room.sdf`

需要添加的插件（示例）：
```xml
<plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
<plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
<plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<plugin filename="ignition-gazebo-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

## 验证方法
```bash
ros2 launch slam_car gazebo.launch.py
ros2 topic list | grep livox
ros2 topic echo /livox_mid360/points --once
```

## 参考资料（线上）
1. `ros_gz_bridge` 官方 README  
   https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md
2. Gazebo Sim 传感器教程（包含 lidar）  
   https://gazebosim.org/docs/garden/sensors/
3. Gazebo Sim 与 ROS 2 集成文档（含桥接说明）  
   https://gazebosim.org/docs/garden/ros2_integration/
