# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Sim_SLAM** is a ROS 2 simulation-based SLAM (Simultaneous Localization and Mapping) system featuring:
- A simulated robot (slam_car) in Gazebo Sim (Gazebo Fortress)
- Livox MID360 lidar sensor emulation via gpu_lidar for point cloud generation
- Graph-based SLAM using lidarslam_ros2 (point clouds only, no IMU data)
- 3D occupancy grid mapping via octomap
- Map visualization and saving capabilities

The workspace is a ROS 2 (Humble) colcon project with multiple integrated packages.

## Build & Development

### Build the workspace
```bash
colcon build --symlink-install
```

### Source the workspace
```bash
source install/setup.bash
```

### Clean build artifacts
```bash
colcon clean workspace --remove-install
```

## Running the System

The SLAM pipeline runs in three sequential launches:

1. **Gazebo Simulator with Robot**
   ```bash
   ros2 launch slam_car gazebo.launch.py
   ```
   - Starts Gazebo Sim with living_room world
   - Spawns slam_car robot and Livox MID360 sensor
   - Bridges ROS 2 ↔ Gazebo topics for sensor data
   - Publishes `/livox_mid360/points` (PointCloud2) and `/livox_mid360` (LaserScan)
   - Publishes odometry at `/odometry` and TF transforms

2. **LidarSLAM Node**
   ```bash
   ros2 launch lidarslam lidarslam.launch.py
   ```
   - Processes point cloud from lidar
   - Outputs pose estimates and map
   - Requires step 1 to be running

3. **OctoMap Server (optional)**
   ```bash
   ros2 launch octomap_server octomap_mapping.launch.xml
   ```
   - Builds 3D occupancy grid from point clouds
   - Optional 3rd step for visualization

### Save & View Maps
```bash
bash scripts/save_map.sh       # Save map to my_map.pgm/my_map.yaml
octovis map.bt                 # Visualize 3D octomap
```

## Architecture

### Core Packages

**slam_car** — Main robot simulation package
- `src/gazebo_wheel_controller.cpp` — Gazebo plugin for wheel actuation
- `src/wheel_vel_publisher.cpp` — Publishes wheel velocities to Gazebo
- `src/keyboard_control.cpp` — Keyboard teleoperation node
- `src/custom_slam_controller.cpp` — ROS 2 component for control logic
- `urdf/slam_car.xacro` — Robot URDF definition (wheels, base, sensor mounts)
- `launch/gazebo.launch.py` — Main Gazebo + bridge configuration
- `world/living_room/` — Gazebo world file with room environment

**lidarslam_ros2** — SLAM algorithm implementation
- `lidarslam/` — Core SLAM logic and pose optimization
- `graph_based_slam/` — Graph optimization for loop closure
- `scanmatcher/` — Point cloud registration
- `lidarslam_msgs/` — Custom ROS 2 message definitions

**Livox Sensor Simulation**
- `livox_laser_simulation_RO2/urdf/mid360.xacro` — Gazebo gpu_lidar definition (replaces old Livox plugin)
- `livox_ros_driver2/` — Real hardware driver (not used in sim)

**octomap** — 3D mapping
- `octomap_mapping/` — Point cloud → occupancy grid converter
- `octomap_ros/`, `octomap_msgs/` — ROS 2 integration

### Message Flow

```
Gazebo Sim (lidar)
       ↓ [Gazebo Transport: LaserScan/PointCloud]
ros_gz_bridge
       ↓ [ROS 2 Topics]
/livox_mid360/points (PointCloud2)
       ↓
lidarslam_ros2 (processes point clouds)
       ↓
Pose estimates + Point cloud map
       ↓ [optional]
octomap_server
       ↓
3D occupancy grid map
```

### Key Gazebo Integration

- **Sensor Bridge** (`gazebo.launch.py`): Converts Gazebo published LaserScan and PointCloud to ROS 2 using `ros_gz_bridge`
- **gpu_lidar** sensor: Renders point clouds using Gazebo's GPU ray caster (1024×64 samples at 100Hz)
- **World Plugins**: Sensors system plugin must be enabled in SDF for lidar to publish data
- **URDF → SDF**: slam_car.xacro is converted to SDF during launch via xacro

### Known Issues & Context

- **Point Cloud Update Rate**: Sensor publishes at 100 Hz; may need tuning for real-time performance
- **Sparse Feature Environment**: living_room world has limited geometric features; SLAM accuracy depends on odometry + features
- **No IMU**: Sensor simulation only includes lidar; any IMU fusion must use external odometry
- **mid360 Mesh Conflicts**: Historical issue with mid360 mesh and xacro file conflicts (resolved with gpu_lidar approach)

## Testing & Debugging

### Monitor key topics
```bash
ros2 topic list                           # See all topics
ros2 topic echo /livox_mid360/points      # Check point cloud data
ros2 topic echo /odometry                 # Check odometry output
ros2 tf2_tools frames_monitor             # Monitor TF tree
```

### Check Gazebo bridge status
```bash
ros2 topic info /livox_mid360/points      # Verify bridge working
```

### View ROS 2 nodes and topic graph
```bash
rqt_graph                                 # Node/topic visualization
```

## Git Workflow

- **Main branch**: stable integration with tested configurations
- **Develop branch**: active development with experimental features
- Current state includes local changes to:
  - `src/slam_car/urdf/slam_car.xacro` — Gazebo sensor config
  - `src/slam_car/CMakeLists.txt` — Build configuration updates
  - `src/slam_car/src/keyboard_control.cpp` — Input handling
  - `scripts/save_map.sh` — Map export script
  - New wheel controller components and test launchers

## Environment & Dependencies

- **ROS 2 Humble** with standard message packages
- **Gazebo Sim (Fortress)** via `gazebo_ros`, `gazebo_dev`, `ros_gz_sim`
- **Point Cloud Library** via PCL (indirect)
- **TF2** for coordinate transforms
- C++17 standard (set in CMakeLists.txt)

## Common Development Tasks

### Add a new ROS 2 node
1. Create source file in `src/slam_car/src/`
2. Add executable target in `CMakeLists.txt`
3. Add to `install()` section
4. Create launch file in `src/slam_car/launch/`
5. Run `colcon build --symlink-install`

### Modify robot URDF
1. Edit `src/slam_car/urdf/slam_car.xacro`
2. Ensure `gazebo.launch.py` xacro path is correct
3. Rebuild: `colcon build`
4. Relaunch `gazebo.launch.py`

### Tune lidar parameters
- Sensor definition: `src/livox_laser_simulation_RO2/urdf/mid360.xacro`
- Update frequency, sampling, range, etc.
- Rebuild and relaunch Gazebo

### Debug sensor data not publishing
1. Verify Sensors plugin enabled in world SDF
2. Check `ros_gz_bridge` command in `gazebo.launch.py`
3. Monitor `/gazebo_ros_state` and TF for position updates
4. Use `ros2 launch slam_car gazebo.launch.py` with output to see bridge logs
