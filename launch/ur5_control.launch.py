import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import FindExecutable


def generate_launch_description():
    # Get URDF via xacro
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_description"),
            "urdf",
            "ur.urdf.xacro"
        ]),
        " ",
        "name:=ur",
        " ",
        "ur_type:=ur5e",
        " ",
        "prefix:=''",
    ])

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

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    return LaunchDescription([
        rviz_node,
        robot_state_pub_node,
        joint_state_publisher_node
        # static_tf,
        # run_move_group_node,
        # ros2_control_node,
    ])
    # + load_controllers
