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
    collision_avoidance_node = Node(
        package="kuka_arm_pkg",
        executable="kuka_collision_avoidance_demo",
        name="kuka_collision_avoidance_demo",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([collision_avoidance_node])

