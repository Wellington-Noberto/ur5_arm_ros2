from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            arguments=['--ros-args',
                '-p',
                PathJoinSubstitution([
                    FindPackageShare("ur5_weaver"),
                    "config",
                    "gz_bridge.yaml"
                ])
            ]
        )
    ])
