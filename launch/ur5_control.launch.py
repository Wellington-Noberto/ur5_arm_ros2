import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ur_moveit_config.launch_common import load_yaml



def generate_launch_description():
    # Get URDF via xacro

    moveit_package = "ur5_weaver"
    description_package = "ur_description"
    ur_type = "ur5e"
    description_file = "ur.urdf.xacro"
    moveit_config_file = "ur.srdf.xacro"
    prefix = ""
    _publish_robot_description_semantic = True

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            "true",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_package), "urdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "config", "kinematics.yaml"]
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_package,
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # Controllers
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            # ompl_planning_pipeline_config,
            # trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # {"use_sim_time": use_sim_time}
        ],
    )


    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur5_weaver"),
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
        parameters=[robot_description],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )


    # ros2_control_node = Node(
	# 	package="controller_manager",
	# 	executable="ros2_control_node",
	# 	parameters=[{"robot_description": robot_description}, robot_controllers],
	# 	output="both",
    # )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )


    return LaunchDescription([
        robot_state_pub_node,
        joint_state_publisher_node,
        move_group_node,
        # color_camera_info_node,
        # static_tf,
        # run_move_group_node,
        # ros2_control_node,
        # arm_controller_spawner,
        # gripper_controller_spawner,
        rviz_node,
        # delay_joint_state_broadcaster_after_robot_controller_spawner
    ])
    # + load_controllers
