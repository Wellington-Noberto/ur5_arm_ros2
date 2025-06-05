# launch/full_sim.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur5_arm"),
        "rviz",
        "view.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["-d", rviz_config_file]
    )



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

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )


    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(spawn_launch)),
        # joint_state_publisher_node,
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     name='camera_bridge',
        #     output='screen',
        #     parameters=[{'ros_gz_bridge': 'camera/image_raw@camera/image_raw@sensor_msgs/msg/Image'}],
        #     remappings=[('/camera/image_raw', '/gazebo/camera/image_raw')]
        # )
    ])
