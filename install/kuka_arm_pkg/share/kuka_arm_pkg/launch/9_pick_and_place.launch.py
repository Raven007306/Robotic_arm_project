from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build MoveIt configurations
    moveit_config = (
        MoveItConfigsBuilder("kuka_arm_moveit", package_name="kuka_arm_moveit")
        .robot_description(file_path="config/kr210_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/kr210_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Launch the collision avoidance demo
    pick_and_place = Node(
        package="kuka_arm_pkg",
        executable="kuka_pick_and_place",
        name="kuka_pick_and_place",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([pick_and_place])
