import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Launch custom world Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare('gazebo_ros').find('gazebo_ros'),
                    'launch/gzserver.launch.py'  # Updated to the correct Gazebo launch file
                )
            ),
            launch_arguments={
                'world_name': LaunchConfiguration('world_name', default=os.path.join(
                    FindPackageShare('kuka_arm_pkg').find('kuka_arm_pkg'),
                    'worlds/cafe_custom.world'
                )),
                'paused': 'false',
                'gui': 'true'
            }.items()
        ),

        # Launch the KUKA arm in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare('kuka_arm_pkg').find('kuka_arm_pkg'),
                    'launch/2_gazebo_kuka_arm.launch.py'  # Ensure this file exists in your package
                )
            )
        ),
    ])
