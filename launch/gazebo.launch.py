# launch/gazebo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    worlds_path = PathJoinSubstitution([
        FindPackageShare('ur5_arm'),
        'worlds',
        'scene.sdf'
    ])

    ur5_arm_path = FindPackageShare('ur5_arm')
    set_gz_sim_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([ur5_arm_path])
    )

    gz_launch_file = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    gz_launch_file = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    return LaunchDescription([
        set_gz_sim_path,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_file),
            launch_arguments={'gz_args': [worlds_path, ' -r']}.items()
        )
    ])
