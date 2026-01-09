// Copyright (c) 2025, Wellington Araujo.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/move_to.h>

#include "weaver_interfaces/srv/weaver_trajectory.hpp"


#include <moveit/trajectory_processing/iterative_time_parameterization.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api_tutorial");

namespace mtc = moveit::task_constructor;
class MissionPlanner : public rclcpp::Node
{
public:
  MissionPlanner(const rclcpp::NodeOptions &options)
      : Node("mission_planner", options)
      , visual_tools_{nullptr}
      , tf_buffer_(this->get_clock())
      , tf_listener_(tf_buffer_)
  {
    this->get_parameter_or("base_link", base_link_, std::string("base_link"));
    this->get_parameter_or("tool_link", tool_link_, std::string("tool0"));
    this->get_parameter_or("tag_id", tag_id_, std::string("apriltag_0"));
    this->get_parameter_or("planning_group", planning_group_, std::string("ur_manipulator"));
    this->get_parameter_or("x_offset", x_offset_, 0.0);
    this->get_parameter_or("y_offset", y_offset_, 0.0);
    this->get_parameter_or("z_offset", z_offset_, 0.0);
  }

  void init()
  {
    RCLCPP_INFO(this->get_logger(), "Init()");

    while (!this->count_subscribers("/joint_states")) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states publisher...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    client_ = this->create_client<weaver_interfaces::srv::WeaverTrajectory>("trajectory_generator_service");
    // Publisher for displaying planned path
    display_publisher_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local());

    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());

    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
    moveit_cpp_->getPlanningSceneMonitor()->startStateMonitor();
    moveit_cpp_->getPlanningSceneMonitor()->startSceneMonitor();
    moveit_cpp_->getPlanningSceneMonitor()->startWorldGeometryMonitor();

    planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_);
    RCLCPP_INFO(this->get_logger(), "planning_component_");

    // Init Visual
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(this->shared_from_this(),
                      base_link_,
                      rviz_visual_tools::RVIZ_MARKER_TOPIC,
                      moveit_cpp_->getPlanningSceneMonitorNonConst());
    visual_tools_->deleteAllMarkers();
    visual_tools_->loadRemoteControl();

    set_planning_group();
  }

  void set_planning_group()
  {
    RCLCPP_INFO(this->get_logger(), "set_planning_group");

    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
  }

  /**
   * @brief Loads the robot model and check joint group information
   *
   */
  void inspection()
  {
    // Load robot model
    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());
    // A robot state contains the configuration of the robot at a given time (joint positions, vel, etc.)
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    // A JointModelGroup represents the robot model for a given group (set of joints).
    joint_model_group = kinematic_model->getJointModelGroup(planning_group_);
    // Get the names of the joints in the group.
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    // Get the default joint values for the group.
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    // Print joint values of the group.
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }

  /**
   * @brief Get the goal pose, which makes the tool_link aligned with the tag_id frame to then perform the weaving task
   *
   * @return geometry_msgs::msg::PoseStamped
   */
  geometry_msgs::msg::PoseStamped set_goal_pose()
  {
    // Get transforms from base_link to tool_link
    geometry_msgs::msg::TransformStamped base_tool_tf = tf_buffer_.lookupTransform(
        base_link_, tool_link_, tf2::TimePointZero);
    // Get transform from tool_link to tag_id. This transformation is published by the marker_detector node
    geometry_msgs::msg::TransformStamped tool_tag_tf = tf_buffer_.lookupTransform(
        tool_link_, tag_id_, tf2::TimePointZero);

    // Convert to Eigen
    Eigen::Affine3d base_tool_eigen = tf2::transformToEigen(base_tool_tf);
    Eigen::Affine3d tool_goal_eigen = tf2::transformToEigen(tool_tag_tf);
    // This offset is the desired distance from the tool to the weaving board. Otherwise, the tool would collide with the board.
    tool_goal_eigen.translate(Eigen::Vector3d(x_offset_, y_offset_, z_offset_));

    // Get final goal pose in base_link frame
    Eigen::Affine3d base_goal_eigen = base_tool_eigen * tool_goal_eigen;
    geometry_msgs::msg::TransformStamped goal_pose = tf2::eigenToTransform(base_goal_eigen);
    RCLCPP_INFO(this->get_logger(),
                "Transform from base to apriltag_0: translation [%.3f, %.3f, %.3f]",
                goal_pose.transform.translation.x,
                goal_pose.transform.translation.y,
                goal_pose.transform.translation.z);

    // Convert to PoseStamped
    geometry_msgs::msg::PoseStamped pose;
    pose.header = goal_pose.header;
    // Set the position
    pose.pose.position.x = goal_pose.transform.translation.x;
    pose.pose.position.y = goal_pose.transform.translation.y;
    pose.pose.position.z = goal_pose.transform.translation.z;
    // Set the orientation
    pose.pose.orientation = goal_pose.transform.rotation;

    return pose;
  }

  void setup_planning_scene()
  {
    // Add table obstacle
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = base_link_;
    collision_object.id = "table";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.5, 1.0, 0.1 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.6;
    box_pose.position.z = 0.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(collision_object);
  }

  void doTask(mtc::Task& task)
  {

    try
    {
      task.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      return;
    }

    if (!task.plan(5))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }

    // auto solution = task.solutions().front();
    const auto& solution = task.solutions().front();

    auto sequence = std::dynamic_pointer_cast<const mtc::SolutionSequence>(task.solutions().front());
    if (!sequence) {
      RCLCPP_ERROR(LOGGER, "Top-level solution is not a SolutionSequence");
      return;
    }

    for (const auto* sub : sequence->solutions()) {
    auto sub_traj = dynamic_cast<const mtc::SubTrajectory*>(sub);
    if (!sub_traj) continue;

    auto robot_traj = sub_traj->trajectory();
    if (!robot_traj) continue;

    // Get planning group and JMG
    auto jmg = moveit_cpp_->getRobotModel()->getJointModelGroup(planning_group_);
    if (!jmg) {
      RCLCPP_WARN(LOGGER, "Could not find JointModelGroup: %s", planning_group_.c_str());
      continue;
    }

    RCLCPP_INFO(LOGGER, "Group name: %s", jmg->getName().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Tips: %s",
                jmg->getLinkModelNames().back().c_str());


    // Visualize the trajectory line in RViz
    visual_tools_->publishTrajectoryLine(*robot_traj, jmg);

    // Publish full trajectory message
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    robot_traj->getRobotTrajectoryMsg(display_trajectory.trajectory.emplace_back());

    moveit::core::RobotState start_state = robot_traj->getFirstWayPoint();
    moveit::core::robotStateToRobotStateMsg(start_state, display_trajectory.trajectory_start);

    display_publisher_->publish(display_trajectory);
  }
    visual_tools_->trigger();


    // Allow instrospection in Rviz
    task.introspection().publishSolution(*solution);
    // Visualize the trajectory



    auto result = task.execute(*solution);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      return;
    }

    return;
  }

  mtc::Task create_move_to_task(std::string task_name, geometry_msgs::msg::PoseStamped target_pose)
  {
    mtc::Task task;
    task.stages()->setName(task_name);
    task.loadRobotModel(this->shared_from_this());
    task.setProperty("ik_frame", tool_link_);

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(0.02);
    cartesian_planner->setJumpThreshold(0.0);  // Disables
    cartesian_planner->setIKFrame(tool_link_);
    cartesian_planner->setProperty("min_fraction", 0.1);

    // Check if the robot state is complete
    auto state_monitor = moveit_cpp_->getPlanningSceneMonitor()->getStateMonitor();

    RCLCPP_INFO(this->get_logger(), "Waiting for complete robot state...");
    auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);  // Use sim time
    state_monitor->waitForCurrentState(ros_clock->now(), 5.0);

    if (!state_monitor->haveCompleteState()) {
      RCLCPP_ERROR(this->get_logger(), "Joint state is incomplete or missing after timeout.");
      return task;
    }

    // Create stages
    {
      auto current = std::make_unique<mtc::stages::CurrentState>("current_state");
      // TODO: Log current joint values

      task.add(std::move(current));
    }

    // MoveTo Stage
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("goal_pose", cartesian_planner);
      stage->setGroup(planning_group_);
      stage->setIKFrame(tool_link_);

      stage->setGoal(target_pose);
      task.add(std::move(stage));
    }

    return task;
  }


  mtc::Task create_task(std::string task_name, std::vector<geometry_msgs::msg::PoseStamped> waypoints)
  {
    mtc::Task task;
    task.stages()->setName(task_name);
    task.loadRobotModel(this->shared_from_this());
    task.setProperty("ik_frame", tool_link_);

    // Planners
    // auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.2);
    cartesian_planner->setMaxAccelerationScalingFactor(0.2);
    cartesian_planner->setStepSize(0.02);
    cartesian_planner->setJumpThreshold(0.0);  // Disables
    cartesian_planner->setIKFrame(tool_link_);
    cartesian_planner->setProperty("min_fraction", 0.1);

    auto state_monitor = moveit_cpp_->getPlanningSceneMonitor()->getStateMonitor();

    RCLCPP_INFO(this->get_logger(), "Waiting for complete robot state...");
    auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);  // Use sim time
    state_monitor->waitForCurrentState(ros_clock->now(), 5.0);

    if (!state_monitor->haveCompleteState()) {
      RCLCPP_ERROR(this->get_logger(), "Joint state is incomplete or missing after timeout.");
      return task;
    }

    // Create stages
    {
      auto current = std::make_unique<mtc::stages::CurrentState>("current_state");

      // TODO: Log current joint values

      task.add(std::move(current));
    }

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("stage_" + std::to_string(i), cartesian_planner);
      stage->setGroup(planning_group_);
      stage->setIKFrame(tool_link_);

      stage->setGoal(waypoints[i]);
      task.add(std::move(stage));
    }

    return task;
  }

std::vector<geometry_msgs::msg::PoseStamped> call_trajectory_service()
{
    RCLCPP_INFO(this->get_logger(), "Called the trajectory service");

    std::vector<geometry_msgs::msg::PoseStamped> result;

    auto request = std::make_shared<weaver_interfaces::srv::WeaverTrajectory::Request>();


    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          continue;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
    }


  auto future = client_->async_send_request(request);

  while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      // Let executor thread run separately; we just wait for the result
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (rclcpp::ok())
  {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Number of waypoints %ld", response->waypoints.size());
      return response->waypoints;
  } else {
      RCLCPP_ERROR(this->get_logger(), "Service call interrupted");
      return {};
  }
}

  void prompt_moveit()
  {
    visual_tools_->prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    visual_tools_->deleteAllMarkers();
    visual_tools_->trigger();
  }


private:
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
  moveit::core::JointModelGroup* joint_model_group;
  std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

  rclcpp::Client<weaver_interfaces::srv::WeaverTrajectory>::SharedPtr client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher_;


  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_link_, tool_link_, tag_id_, planning_group_;
  double x_offset_, y_offset_, z_offset_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<MissionPlanner>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread([&executor]()
  {
    executor.spin();
  });

  node->init();
  node->inspection();


  std::vector<geometry_msgs::msg::PoseStamped> waypoints_weaver;
  waypoints_weaver = node->call_trajectory_service();

  // [0.445, -0.011, -0.090]

  // Add obstacles
  node->setup_planning_scene();

  // Task homing
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "base_link";
  target_pose.pose.position.x = 0.1;
  target_pose.pose.position.y = 0.6;
  target_pose.pose.position.z = 0.3;
  target_pose.pose.orientation.x = 1.0;
  target_pose.pose.orientation.w = 0.0;
  mtc::Task task_homing = node->create_move_to_task("homing pose", target_pose);
  node->doTask(task_homing);

  node->prompt_moveit();

  // Task approach
  geometry_msgs::msg::PoseStamped board_pose = node->set_goal_pose();
  mtc::Task task_apprach = node->create_move_to_task("approach board", board_pose);
  node->doTask(task_apprach);

  node->prompt_moveit();

  // Weaver task
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  waypoints.push_back(board_pose);
  for (const auto &pose : waypoints_weaver)
  {
    geometry_msgs::msg::PoseStamped pose_wp = board_pose;
    pose_wp.pose.position.x += pose.pose.position.x;
    pose_wp.pose.position.y += pose.pose.position.y;
    waypoints.push_back(pose_wp);
  }
  mtc::Task task_weaver = node->create_task("weaver task", waypoints);
  node->doTask(task_weaver);


  std::cin.get();

  executor.cancel();
  if (executor_thread.joinable())
  {
    executor_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
