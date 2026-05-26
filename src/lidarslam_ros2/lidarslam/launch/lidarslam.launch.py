"""
LiDAR SLAM launch configuration for slam_car.

This launch file:
  - Starts scan matcher for point cloud registration
  - Publishes static transforms for sensor frames
  - Runs graph-based SLAM for loop closure detection
  - Launches RViz for visualization
"""

import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for LiDAR SLAM with slam_car."""

    # ============================================================================
    # Configuration Paths
    # ============================================================================
    main_param_dir = LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'
        )
    )

    rviz_param_dir = LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'
        )
    )

    # ============================================================================
    # Scan Matcher Node (Point Cloud Registration)
    # ============================================================================
    mapping_node = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[
            ('/input_cloud', '/livox_mid360/points'),
        ],
        output='screen',
    )

    # ============================================================================
    # Static Transform Publishers
    # ============================================================================
    # Map base_link to velodyne (for compatibility)
    tf_base_to_velodyne = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',      # translation (x, y, z)
            '0', '0', '0', '1', # rotation as quaternion (x, y, z, w)
            'base_link',         # parent frame
            'velodyne'           # child frame
        ],
        output='screen',
    )

    # Handle Gazebo-style frame naming (slam_car/base_footprint/livox_mid360 -> livox_mid360)
    # This bridges the gap between Gazebo's scoped frame names and ROS TF
    tf_gazebo_sensor_bridge = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',      # translation (x, y, z)
            '0', '0', '0', '1', # rotation as quaternion (x, y, z, w)
            'livox_mid360',         # parent frame (ROS TF name)
            'slam_car/base_footprint/livox_mid360'  # child frame (Gazebo scoped name)
        ],
        output='screen',
    )

    # ============================================================================
    # Graph-Based SLAM Node (Loop Closure Detection)
    # ============================================================================
    graphbasedslam_node = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen',
    )

    # ============================================================================
    # RViz Visualization
    # ============================================================================
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir],
        output='screen',
    )

    # ============================================================================
    # Launch Description
    # ============================================================================
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'
        ),
        DeclareLaunchArgument(
            'rviz_param_dir',
            default_value=rviz_param_dir,
            description='Full path to RViz configuration file to load'
        ),
        tf_base_to_velodyne,
        tf_gazebo_sensor_bridge,
        mapping_node,
        graphbasedslam_node,
        rviz_node,
    ])