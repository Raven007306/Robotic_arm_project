from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build MoveIt configurations
    moveit_config = (
        MoveItConfigsBuilder("kuka_arm", package_name="kuka_arm_moveit")
        .robot_description(file_path="config/kr210_arm.urdf.xacro")  # Path to KUKA URDF
        .robot_description_semantic(file_path="config/kr210_arm.srdf")  # Path to KUKA SRDF
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # KUKA controllers
        .planning_pipelines()  # Use default pipelines
        .to_moveit_configs()
    )

    # Launch the Move Group node with proper frame remapping
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"robot_description_planning_frame": "base_footprint"},  # Force planning in base_footprint
            {"octomap": False},  # Disable Octomap to avoid occupancy map errors
        ],
        remappings=[
            ("/base_world_joint", "/base_footprint"),  # Map frames dynamically
        ],
    )

    # Launch the MTC task node
    task_node = Node(
        package="mtc_tutorial",
        executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    return LaunchDescription([move_group_node, task_node])

