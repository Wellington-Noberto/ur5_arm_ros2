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
    ur_description_config_package = "ur_description"
    ur_type = "ur5e"
    description_file = "ur.urdf.xacro"
    moveit_config_file = "ur.srdf.xacro"
    controllers_file = "ur_controllers.yaml"
    world_file = "scene.sdf"
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = True
    _publish_robot_description_semantic = True

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_config_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_config_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_config_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(ur_description_config_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = ParameterValue(
        Command(
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


            ]
        ),
        value_type=str
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare(package_name), "urdf", moveit_config_file]
                ),
                " ",
                "name:=",
                "ur",
                " ",
                "prefix:=",
                "",
                " ",
            ]
        ),
        value_type=str
    )

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "kinematics.yaml"]
    )

    publish_robot_description_semantic = {
        "publish_robot_description_semantic": _publish_robot_description_semantic
    }

    # controllers_path = PathJoinSubstitution([
    #     FindPackageShare(package_name),
    #     "config",
    #     "ur_controllers.yaml"
    # ])

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            package_name,
            os.path.join("config", "joint_limits.yaml"),
        )
    }


    # Planning Configuration
    ompl_planning_pipeline_config = {
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Configuration
    controllers_yaml = load_yaml(package_name, "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    # change_controllers = "true"
    # if change_controllers == "true":
    #     controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    #     controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
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

    # moveit_config = (
    #     MoveItConfigsBuilder(package_name)
    #     .robot_description(file_path="config/ur.urdf.xacro")
    #     .robot_description_semantic(file_path="config/panda.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .planning_pipelines(pipelines=["ompl"])
    #     .to_moveit_configs()
    # )



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
            {"planning_pipelines": ["ompl"]},
            {"ompl.planning_plugins": ["ompl_interface/OMPLPlanner"]},
            {"use_sim_time": use_sim_time},
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
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            moveit_controllers,
            {"planning_pipelines": ["ompl"]},
            {"use_sim_time": use_sim_time}
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )


    ### Controllers
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

    ### Gazebo nodes

    world_config_file = PathJoinSubstitution([
        FindPackageShare("ur5_weaver"),
        "worlds",
        "scene.sdf"
    ])

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [' -r -v 1 ', world_config_file])],
    )


    # Spawn robot
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur', '-topic', 'robot_description',
            '-x', '1.9',                   # Desired X position
            '-y', '-2.0',                  # Desired Y position
            '-z', '0.85',                  # Desired Z position
            '-Y', '0.0'],
        output='screen'
    )

    # Gazebo Bridge
    bridge_config_path = PathJoinSubstitution([
        FindPackageShare(package_name),
        "config",
        "gz_bridge.yaml"
    ])

    gz_parameters_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', ['config_file:=', bridge_config_path]
        ]
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
        move_group_node,
        ur_controller_spawner,
        gazebo,
        gazebo_spawn_robot,

        # static_tf,
        rviz_node,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        # joint_state_broadcaster_spawner,
        gz_parameters_bridge
    ])
    # + load_controllers
