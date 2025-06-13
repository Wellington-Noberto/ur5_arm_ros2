# launch/full_sim.launch.py
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    robot_name = "ur5e"
    ur_moveit_config_path = Path(get_package_share_directory("ur_moveit_config"))
    controller_yaml_path = ur_moveit_config_path / "config" / "controllers.yaml"

    moveit_config = (
        MoveItConfigsBuilder(robot_name=robot_name, package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", mappings={"name": robot_name})
        .trajectory_execution(file_path=str(controller_yaml_path))
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

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
        run_move_group_node

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
