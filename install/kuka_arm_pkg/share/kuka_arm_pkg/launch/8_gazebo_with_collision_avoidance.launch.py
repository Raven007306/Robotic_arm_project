from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Path to Robot's Xacro File
    pkg_path = get_package_share_directory("kuka_arm_pkg")
    xacro_file = os.path.join(pkg_path, 'urdf', 'kr210.urdf.xacro')

    # Processing Xacro File
    xacro_parser = xacro.parse(open(xacro_file))
    xacro.process_doc(xacro_parser)

    # Feeding URDF to ROS
    parameters = {'robot_description': xacro_parser.toxml()}

    # Gazebo Node
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={"verbose": "true"}.items()
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[parameters]
    )

    # Spawn Robot in Gazebo
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'kuka_arm'
        ],
        output='screen'
    )

    # Load and Activate Controllers for Gazebo
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_kuka_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'kuka_arm_controller'],
        output='screen'
    )

    load_kuka_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'kuka_gripper_controller'],
        output='screen'
    )

    # Include MoveIt and RViz
    kuka_moveit_path = os.path.join(
        get_package_share_directory('kuka_arm_moveit'),
        'launch', 'demo.launch.py'
    )

    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(kuka_moveit_path),
        launch_arguments={}
    )

    # Collision Avoidance Script
    collision_avoidance_script = Node(
        package='kuka_arm_pkg',
        executable='kuka_collision_avoidance_demo',
        output='screen'
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters = [{'robot_description': xacro_parser.toxml()},
        os.path.join(pkg_path, "config", "kuka_arm_controllers.yaml")
    ],
    output="screen",
    )   

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_robot_node,
        load_joint_state_controller,
        load_kuka_arm_controller,
        load_kuka_gripper_controller,
        moveit_demo_launch,
        ros2_control_node,
        collision_avoidance_script
    ])
