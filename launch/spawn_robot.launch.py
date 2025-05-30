# launch/spawn_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    robot_description = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur5_arm"),
            "urdf",
            "ur5e_joint_limited_robot.urdf.xacro"
        ]),
        " ",
        "name:=ur",
        " ",
        "ur_type:=ur5e",
        " ",
        "tf_prefix:=''",
    ]),
    value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'ur5_weaver', '-topic', 'robot_description'],
            output='screen'
        )
    ])
