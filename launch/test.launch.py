from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml
import os

def generate_launch_description():
    package_name = "ur5_weaver"
    ur_description_config_package = "ur_description"
    ur_type = "ur5e"
    description_file = "ur.urdf.xacro"
    moveit_config_file = "ur.srdf.xacro"

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

    # Planning Configuration
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            package_name,
            os.path.join("config", "joint_limits.yaml"),
        )
    }

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
    change_controllers = "true"
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }


    tutorial_node = Node(
        package=package_name,
        executable="motion_planning_api_tutorial",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            moveit_controllers,
            {"planning_pipelines": ["ompl"]},
            {"ompl.planning_plugins": ["ompl_interface/OMPLPlanner"]},
        ],
    )

    return LaunchDescription([
        tutorial_node
    ])
