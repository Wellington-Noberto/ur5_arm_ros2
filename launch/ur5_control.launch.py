import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # Get URDF via xacro
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur_description"),
            "urdf",
            "ur_mocked.urdf.xacro"
        ]),
        " ",
        "name:=ur",
        " ",
        "ur_type:=ur5e",
        " ",
        "prefix:=''",
    ])

    # ToDO: Create a xacro that contains the model and control

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ur5_arm"),
        "config",
        "ros2_controllers.yaml",
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

    ros2_control_node = Node(
		package="controller_manager",
		executable="ros2_control_node",
		parameters=[{"robot_description": robot_description}, robot_controllers],
		output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur5_arm_controller", "--param-file", robot_controllers],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    return LaunchDescription([
        robot_state_pub_node,
        # joint_state_publisher_node,
        # static_tf,
        # run_move_group_node,
        ros2_control_node,
        robot_controller_spawner,
        # joint_state_broadcaster_spawner,
        rviz_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner
    ])
    # + load_controllers
