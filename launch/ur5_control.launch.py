import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import FindExecutable, Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ur_moveit_config.launch_common import load_yaml
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    # Get URDF via xacro

    package_name = "ur5_weaver"
    runtime_config_package = "ur_robot_driver"
    ur_type = "ur5e"
    description_file = "ur.urdf.xacro"
    moveit_config_file = "ur.srdf.xacro"
    controllers_file = "ur_controllers.yaml"
    tf_prefix = LaunchConfiguration("tf_prefix")

    _publish_robot_description_semantic = True


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name), "urdf", description_file]),
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            "",
            " ",
            "use_fake_hardware:=",
            "true",
            " ",
            "sim_gazebo:=",
            "true",
            " ",

        ]
    )
    robot_description = {"robot_description": robot_description_content}


    # controllers_path = PathJoinSubstitution([
    #     FindPackageShare(package_name),
    #     "config",
    #     "ur_controllers.yaml"
    # ])



    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur5_weaver"),
        "rviz",
        "view.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            # robot_description_semantic,
            # ompl_planning_pipeline_config,
            # robot_description_kinematics,
            # robot_description_planning
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )


    # Controllers
    ur_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
        ,
    )

    # Gazebo nodes
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 empty.sdf'])]
    )


    # Spawn robot
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur', '-topic', 'robot_description'],
        output='screen'
    )


    ###
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ur_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    ##
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )


    return LaunchDescription(
        declared_arguments + [
        robot_state_pub_node,
        # move_group_node,
        ur_controller_spawner,
        gazebo,
        gazebo_spawn_robot,

        # static_tf,
        rviz_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner
    ])
    # + load_controllers
