import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro


def generate_launch_description():
    # Get default path
    robot_name_in_model = "slam_car"
    urdf_tutorial_path = get_package_share_directory('slam_car')
    default_model_path = os.path.join(
        urdf_tutorial_path, 'urdf', 'slam_car.xacro')

    # Read XACRO file and convert to URDF
    doc = xacro.parse(open(default_model_path))
    xacro.process_doc(doc)
    robot_description = doc.toprettyxml(indent='  ')

    #     # Read URDF file content
    # with open(default_model_path, 'r') as urdf_file:
    #     robot_description = urdf_file.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    package_world = os.path.join(urdf_tutorial_path, 'world', 'living_room', 'living_room.sdf')

    default_world_path = package_world

    declare_world_arg = launch.actions.DeclareLaunchArgument(
        'world', default_value=default_world_path,
        description='Full path to world model file to load')

    # Include another launch file for Gazebo and pass the world file
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments={'world': launch.substitutions.LaunchConfiguration('world')}.items()
    )

    # Include another launch file for Gazebo
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
    )

    # Request Gazebo to spawn the robot
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', robot_name_in_model,
            '-x', '0.0',  # X position
            '-y', '1.0',  # Y position
            '-z', '0.0',  # Z position (height)
            '-R', '0.0',  # Roll
            '-P', '0.0',  # Pitch
            '-Y', '0.0'   # Yaw
        ])

    return launch.LaunchDescription([
        robot_state_publisher_node,
        declare_world_arg,
        launch_gazebo,
        spawn_entity_node
    ])