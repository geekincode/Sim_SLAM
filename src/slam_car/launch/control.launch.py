import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_slam_car = get_package_share_directory('slam_car')

    # Create our controller node
    controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_wheel_controller'],
        output='screen'
    )

    # Create the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(
                os.path.join(pkg_slam_car, 'urdf', 'slam_car.xacro')
            ).read()
        }]
    )

    # Create the joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Create the joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        controller,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_broadcaster_spawner
    ])