import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import FindExecutable, Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ur_moveit_config.launch_common import load_yaml
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    # Get URDF via xacro

    moveit_package = "ur5_weaver"
    description_package = moveit_package
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
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
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
            "sim_ignition:=",
            "false",
            " ",

        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(moveit_package), "config", controllers_file]
    )

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
            "tf_prefix:=",
            tf_prefix,
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
    controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    # Parameters
    trajectory_execution = {
    "moveit_manage_controllers": False,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
    # Execution time monitoring can be incompatible with the scaled JTC
    "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

     # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)


    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            ur_type + "_update_rate.yaml",
        ]
    )

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
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            # {"use_sim_time": use_sim_time}
        ],
    )

    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare(moveit_package),
        "config",
        "ur_controllers.yaml"
    ])

    ## Control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    ur_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
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
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning
        ]
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




    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

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
        control_node,
        ur_controller_spawner,
        # color_camera_info_node,
        # static_tf,
        # run_move_group_node,
        # ros2_control_node,
        # arm_controller_spawner,
        # gripper_controller_spawner,
        rviz_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner
    ])
    # + load_controllers
