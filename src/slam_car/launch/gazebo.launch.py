import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
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

    def _find_slam_car_resource_root():
        candidates = [
            os.path.dirname(urdf_tutorial_path),
            os.path.join(os.getcwd(), 'src'),
            os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        ]
        for root in candidates:
            if os.path.isdir(os.path.join(root, 'slam_car', 'meshes')):
                return root
        return os.path.dirname(urdf_tutorial_path)

    slam_car_resource_root = _find_slam_car_resource_root()
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    ign_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    declare_world_arg = DeclareLaunchArgument(
        'world', default_value=default_world_path,
        description='Full path to world model file to load')

    declare_spawn_delay_arg = DeclareLaunchArgument(
        'spawn_delay', default_value='10.0',
        description='Seconds to wait before spawning the robot')

    # Gazebo Sim (ros_gz)
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items(),
    )

    # Spawn the robot
    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name_in_model,
            '-x', '0.0',  # X position
            '-y', '1.0',  # Y position
            '-z', '0.0',  # Z position (height)
            '-R', '0.0',  # Roll
            '-P', '0.0',  # Pitch
            '-Y', '0.0'   # Yaw
        ],
        output='screen')

    ros_gz_bridge_node = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/livox_mid360@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/livox_mid360/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen')

    return launch.LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join([slam_car_resource_root, gz_resource_path]) if gz_resource_path else slam_car_resource_root
        ),
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            os.pathsep.join([slam_car_resource_root, ign_resource_path]) if ign_resource_path else slam_car_resource_root
        ),
        robot_state_publisher_node,
        declare_world_arg,
        declare_spawn_delay_arg,
        launch_gazebo,
        ros_gz_bridge_node,
        TimerAction(
            period=LaunchConfiguration('spawn_delay'),
            actions=[spawn_entity_node]
        )
    ])