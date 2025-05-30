# launch/full_sim.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('ur5_arm'),
        'launch',
        'gazebo.launch.py'
    ])

    spawn_launch = PathJoinSubstitution([
        FindPackageShare('ur5_arm'),
        'launch',
        'spawn_robot.launch.py'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(spawn_launch)),
    ])
