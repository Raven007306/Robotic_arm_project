from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    package_share_path = FindPackageShare('kuka_arm_pkg')
    urdf_file = PathJoinSubstitution([package_share_path, 'urdf', 'kr210.urdf.xacro'])
    world_file = PathJoinSubstitution([package_share_path, 'worlds', 'cafe_custom.world'])

    # Node: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
    )

    # Node: Spawn Entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=['-entity', 'kr210', '-topic', '/robot_description'],
        output='screen',
    )

    # Gazebo Server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # Gazebo Client (optional, for visualization)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
        )
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_client,  # Comment this out if running in headless mode
        robot_state_publisher_node,
        spawn_entity_node,
    ])

