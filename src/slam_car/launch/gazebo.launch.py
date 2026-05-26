"""
ROS 2 Gazebo Sim launch configuration for slam_car robot.

This launch file:
  - Starts Gazebo Sim with the specified world file
  - Publishes robot state and URDF description
  - Spawns the robot into the simulation with configurable delay
  - Bridges ROS 2 and Gazebo topics for sensors
"""

import os
import xacro
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration


def _find_slam_car_resource_root(urdf_tutorial_path):
    """Find the root directory containing slam_car resources (meshes, etc.)."""
    candidates = [
        os.path.dirname(urdf_tutorial_path),
        os.path.join(os.getcwd(), 'src'),
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
    ]
    for root in candidates:
        if os.path.isdir(os.path.join(root, 'slam_car', 'meshes')):
            return root
    return os.path.dirname(urdf_tutorial_path)


def _load_robot_description(urdf_xacro_path):
    """Load and process XACRO file to generate URDF description."""
    with open(urdf_xacro_path, 'r') as f:
        doc = xacro.parse(f)
    xacro.process_doc(doc)
    return doc.toprettyxml(indent='  ')


def _create_resource_path_env(resource_root, existing_path_env_var):
    """Create resource path environment variable by prepending robot root to existing paths."""
    existing_path = os.environ.get(existing_path_env_var, '')
    if existing_path:
        return os.pathsep.join([resource_root, existing_path])
    return resource_root


def generate_launch_description():
    """Generate launch description for Gazebo Sim with slam_car robot."""
    
    # ============================================================================
    # Configuration Constants
    # ============================================================================
    ROBOT_NAME = "slam_car"
    WORLD_SUBDIR = os.path.join('world', 'living_room')
    WORLD_FILENAME = 'living_room.sdf'
    URDF_SUBDIR = 'urdf'
    URDF_FILENAME = 'slam_car.xacro'
    
    LIVOX_LASER_TOPIC = '/livox_mid360'
    LIVOX_POINTS_TOPIC = '/livox_mid360/points'
    
    # Default spawn position and orientation (x, y, z, roll, pitch, yaw)
    DEFAULT_SPAWN_POSITION = ['0.0', '1.0', '0.0']
    DEFAULT_SPAWN_ORIENTATION = ['0.0', '0.0', '0.0']
    
    # ============================================================================
    # Package Paths
    # ============================================================================
    slam_car_dir = get_package_share_directory('slam_car')
    urdf_path = os.path.join(slam_car_dir, URDF_SUBDIR, URDF_FILENAME)
    world_path = os.path.join(slam_car_dir, WORLD_SUBDIR, WORLD_FILENAME)
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    
    # ============================================================================
    # Robot Description
    # ============================================================================
    robot_description = _load_robot_description(urdf_path)
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    
    # ============================================================================
    # Gazebo Resource Path Setup
    # ============================================================================
    slam_car_resource_root = _find_slam_car_resource_root(slam_car_dir)
    gz_resource_path = _create_resource_path_env(slam_car_resource_root, 'GZ_SIM_RESOURCE_PATH')
    ign_resource_path = _create_resource_path_env(slam_car_resource_root, 'IGN_GAZEBO_RESOURCE_PATH')
    
    # ============================================================================
    # Launch Arguments
    # ============================================================================
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to world SDF file to load'
    )
    
    declare_spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay',
        default_value='10.0',
        description='Seconds to wait before spawning robot (allows Gazebo to fully initialize)'
    )
    
    declare_spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value=DEFAULT_SPAWN_POSITION[0],
        description='Robot X spawn position (meters)'
    )
    
    declare_spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value=DEFAULT_SPAWN_POSITION[1],
        description='Robot Y spawn position (meters)'
    )
    
    declare_spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value=DEFAULT_SPAWN_POSITION[2],
        description='Robot Z spawn position (meters)'
    )
    
    declare_spawn_roll_arg = DeclareLaunchArgument(
        'spawn_roll',
        default_value=DEFAULT_SPAWN_ORIENTATION[0],
        description='Robot roll orientation (radians)'
    )
    
    declare_spawn_pitch_arg = DeclareLaunchArgument(
        'spawn_pitch',
        default_value=DEFAULT_SPAWN_ORIENTATION[1],
        description='Robot pitch orientation (radians)'
    )
    
    declare_spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value=DEFAULT_SPAWN_ORIENTATION[2],
        description='Robot yaw orientation (radians)'
    )
    
    # ============================================================================
    # Gazebo Sim Launcher
    # ============================================================================
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items(),
    )
    
    # ============================================================================
    # Robot Spawner Node
    # ============================================================================
    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', ROBOT_NAME,
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', LaunchConfiguration('spawn_z'),
            '-R', LaunchConfiguration('spawn_roll'),
            '-P', LaunchConfiguration('spawn_pitch'),
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
        output='screen',
    )
    
    # ============================================================================
    # ROS 2 <-> Gazebo Bridge Node
    # ============================================================================
    # Bridges laser scan and point cloud topics between ROS 2 and Gazebo Sim
    ros_gz_bridge_node = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'{LIVOX_LASER_TOPIC}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            f'{LIVOX_POINTS_TOPIC}@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )
    
    # ============================================================================
    # Launch Description Assembly
    # ============================================================================
    return launch.LaunchDescription([
        # Log configuration
        LogInfo(msg=f"Starting slam_car simulation with world: {world_path}"),
        LogInfo(msg=f"Resource path: {slam_car_resource_root}"),
        
        # Environment setup
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        
        # Launch arguments
        declare_world_arg,
        declare_spawn_delay_arg,
        declare_spawn_x_arg,
        declare_spawn_y_arg,
        declare_spawn_z_arg,
        declare_spawn_roll_arg,
        declare_spawn_pitch_arg,
        declare_spawn_yaw_arg,
        
        # Nodes
        robot_state_publisher_node,
        launch_gazebo,
        ros_gz_bridge_node,
        
        # Delayed spawn
        TimerAction(
            period=LaunchConfiguration('spawn_delay'),
            actions=[spawn_entity_node],
        ),
    ])