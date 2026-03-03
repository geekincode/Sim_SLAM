import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    # ===== 路径 =====
    slam_car_dir = get_package_share_directory('slam_car')
    cartographer_config_dir = os.path.join(slam_car_dir, 'config', 'cartographer')

    default_model_path = os.path.join(slam_car_dir, 'urdf', 'slam_car.xacro')
    doc = xacro.parse(open(default_model_path))
    xacro.process_doc(doc)
    robot_description = doc.toprettyxml(indent='  ')

    # ===== 1. Robot State Publisher =====
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # ===== 2. Gazebo =====
    world_path = os.path.join(slam_car_dir, 'world', 'living_room', 'living_room.sdf')
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_path}.items()
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'slam_car',
            '-x', '0.0', '-y', '1.0', '-z', '0.0'
        ]
    )

    # ===== 3. PointCloud2 → LaserScan =====
    # Mid-360 仿真输出话题通常为 /mid360 或 /livox/lidar，需根据实际检查
    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.5,
            'max_height': 0.6,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00436,
            'scan_time': 0.1,
            'range_min': 0.15,
            'range_max': 40.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'use_sim_time': True,
        }],
        remappings=[
            ('cloud_in', '/livox_mid360_PointCloud2'),     # ← 改为您实际的点云话题名
            ('scan', '/scan'),
        ]
    )

    # ===== 4. Cartographer =====
    cartographer_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'mid360_2d.lua'
        ],
        remappings=[
            ('scan', '/scan'),
        ]
    )

    # ===== 5. Occupancy Grid（生成 2D 栅格地图）=====
    occupancy_grid_node = launch_ros.actions.Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05
        }]
    )

    # ===== 6. RViz =====
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ===== 7. 键盘遥控 =====
    teleop_node = launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',  # 在新终端窗口打开
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    return launch.LaunchDescription([
        # robot_state_publisher_node,
        # launch_gazebo,
        spawn_entity,
        pointcloud_to_laserscan_node,
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
        # teleop_node,
    ])