/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"
#include <tf2_eigen/tf2_eigen.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_api_tutorial");

class MissionPlanner : public rclcpp::Node
{
public:
  MissionPlanner(const rclcpp::NodeOptions &options)
      : Node("mission_planner", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("base_link", "base_link");
    this->declare_parameter<std::string>("tool_link", "tool0");
    this->declare_parameter<std::string>("tag_id", "apriltag_0");
    this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    this->declare_parameter<double>("x_offset", 0.2);
    this->declare_parameter<double>("y_offset", 0.0);
    this->declare_parameter<double>("z_offset", 0.0);

    this->get_parameter("base_link", base_link_);
    this->get_parameter("tool_link", tool_link_);
    this->get_parameter("tag_id", tag_id_);
    this->get_parameter("planning_group", planning_group_);
    this->get_parameter("x_offset", x_offset_);
    this->get_parameter("y_offset", y_offset_);
    this->get_parameter("z_offset", z_offset_);
  }

  void init()
  {
    RCLCPP_INFO(this->get_logger(), "Init");

    while (!this->count_subscribers("/joint_states")) {
      RCLCPP_INFO(this->get_logger(), "Waiting for /joint_states publisher...");
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(this->shared_from_this());
    RCLCPP_INFO(this->get_logger(), "moveit_cpp");
    // moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService(false);
    // moveit_cpp_->getPlanningSceneMonitor()->setPlanningSceneMonitorEnabled(false);

    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
    moveit_cpp_->getPlanningSceneMonitor()->startStateMonitor();
    moveit_cpp_->getPlanningSceneMonitor()->startSceneMonitor();
    moveit_cpp_->getPlanningSceneMonitor()->startWorldGeometryMonitor();

    planning_component_ = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_, moveit_cpp_);
    RCLCPP_INFO(this->get_logger(), "planning_component_");

    set_planning_group();
  }

  void set_planning_group()
  {
    RCLCPP_INFO(this->get_logger(), "set_planning_group");

    moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
    // moveit_cpp_->getPlanningSceneMonitor()->startWorldGeometryMonitor();

    // planning_component_->setStartStateToCurrentState();



  }

  void inspection()
  {
    robot_model_loader::RobotModelLoader robot_model_loader(this->shared_from_this());
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    RCLCPP_INFO(LOGGER, "Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup(planning_group_);

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }

  void set_goal_pose()
  {

    geometry_msgs::msg::TransformStamped base_tool_tf = tf_buffer_.lookupTransform(
        base_link_, tool_link_, tf2::TimePointZero);

    geometry_msgs::msg::TransformStamped tool_tag_tf = tf_buffer_.lookupTransform(
        tool_link_, tag_id_, tf2::TimePointZero);

    //
    Eigen::Affine3d base_tool_eigen = tf2::transformToEigen(base_tool_tf);
    Eigen::Affine3d tool_goal_eigen = tf2::transformToEigen(tool_tag_tf);
    tool_goal_eigen.translate(Eigen::Vector3d(x_offset_, y_offset_, z_offset_));
    tool_goal_eigen.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));

    Eigen::Affine3d base_goal_eigen = base_tool_eigen * tool_goal_eigen;

    geometry_msgs::msg::TransformStamped goal_pose = tf2::eigenToTransform(base_goal_eigen);

    RCLCPP_INFO(this->get_logger(),
                "Transform from base to apriltag_0: translation [%.3f, %.3f, %.3f]",
                goal_pose.transform.translation.x,
                goal_pose.transform.translation.y,
                goal_pose.transform.translation.z);
  }

  void plan_and_execute(std::vector<geometry_msgs::msg::Pose> waypoints)
  {
    RCLCPP_INFO(this->get_logger(), "Setting target pose...");

    // [0.445, -0.011, -0.090]



    auto state_monitor = moveit_cpp_->getPlanningSceneMonitor()->getStateMonitor();

    RCLCPP_INFO(this->get_logger(), "Waiting for complete robot state...");
    auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);  // Use sim time
    state_monitor->waitForCurrentState(ros_clock->now(), 5.0);


    if (!state_monitor->haveCompleteState()) {
      RCLCPP_ERROR(this->get_logger(), "Joint state is incomplete or missing after timeout.");
      return;
    }




    // geometry_msgs::msg::Pose pose2 = pose1;
    // pose2.position.x += 0.1;
    // waypoints.push_back(pose2);

    // geometry_msgs::msg::Pose pose3 = pose2;
    // pose3.position.z += 0.1;
    // waypoints.push_back(pose3);

    // Set start state from current planning scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = base_link_;
    collision_object.id = "table";

    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.5, 1.0, 0.1 };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.5;
    box_pose.position.z = 0.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitorNonConst());
      scene->processCollisionObjectMsg(collision_object);
    }


    auto planning_scene = moveit_cpp_->getPlanningSceneMonitor()->getPlanningScene();
    moveit::core::RobotState current_state = planning_scene->getCurrentState();
    planning_component_->setStartState(current_state);


    for (size_t i = 0; i < waypoints.size(); ++i) {
      geometry_msgs::msg::PoseStamped goal_pose;
      goal_pose.header.frame_id = "base_link";
      goal_pose.pose = waypoints[i];

      RCLCPP_INFO(this->get_logger(), "Planning to waypoint %ld...", i + 1);

      planning_component_->setGoal(goal_pose, tool_link_);
      auto plan = planning_component_->plan();

      if (plan) {
        RCLCPP_INFO(this->get_logger(), "Planning to waypoint %ld succeeded, executing...", i + 1);
        planning_component_->execute();
        // Optionally: wait for execution to complete, or add a small delay
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning to waypoint %ld failed.", i + 1);
        break;  // Stop if one fails
      }

      // Update the start state to the new goal for next iteration
      planning_component_->setStartStateToCurrentState();
      set_goal_pose();
    }

  }

  // 1 - Go to pose and wait for detection

  // 2 - Get the transformation from base to tool

  // 3 - Set the goal pose

private:
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
  moveit::core::JointModelGroup* joint_model_group;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string base_link_, tool_link_, tag_id_, planning_group_;
  double x_offset_, y_offset_, z_offset_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true); // ✅ this is key
  auto node = std::make_shared<MissionPlanner>(options);

  node->init();
  node->inspection();

  // [0.445, -0.011, -0.090]
  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = base_link_;
  target_pose.pose.position.x = 0.1;
  target_pose.pose.position.y = 0.7;
  target_pose.pose.position.z = 0.3;
  target_pose.pose.orientation.x = 1.0;
  target_pose.pose.orientation.w = 0.0;

  waypoints.push_back(target_pose.pose);

  node->plan_and_execute(waypoints);



  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// // Visualization
// // ^^^^^^^^^^^^^
// // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
// // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
// namespace rvt = rviz_visual_tools;
// moveit_visual_tools::MoveItVisualTools visual_tools(motion_planning_api_tutorial_node, "shoulder_link",
//                                                     "move_group_tutorial", move_group.getRobotModel());
// visual_tools.enableBatchPublishing();
// visual_tools.deleteAllMarkers();  // clear all old markers
// visual_tools.trigger();

// /* Remote control is an introspection tool that allows users to step through a high level script
//    via buttons and keyboard shortcuts in RViz */
// visual_tools.loadRemoteControl();

// /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
// Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
// text_pose.translation().z() = 1.75;
// visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

// /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
// visual_tools.trigger();

// /* We can also use visual_tools to wait for user input */
// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

// // Pose Goal
// // ^^^^^^^^^
// // We will now create a motion plan request for the arm of the Panda
// // specifying the desired pose of the end-effector as input.
// visual_tools.trigger();
// planning_interface::MotionPlanRequest req;
// planning_interface::MotionPlanResponse res;
// geometry_msgs::msg::PoseStamped pose;
// pose.header.frame_id = "shoulder_link";
// pose.pose.position.x = 0.8;
// pose.pose.position.y = 0.8;
// pose.pose.position.z = 0.75;
// pose.pose.orientation.w = 1.0;

// // A tolerance of 0.01 m is specified in position
// // and 0.01 radians in orientation
// std::vector<double> tolerance_pose(3, 0.01);
// std::vector<double> tolerance_angle(3, 0.01);

// // We will create the request as a constraint using a helper function available
// // from the
// // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.hpp>`
// // package.
// moveit_msgs::msg::Constraints pose_goal =
//     kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);

// req.group_name = PLANNING_GROUP;
// req.goal_constraints.push_back(pose_goal);

// // Define workspace bounds
// req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
//     req.workspace_parameters.min_corner.z = -5.0;
// req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
//     req.workspace_parameters.max_corner.z = 5.0;

// // We now construct a planning context that encapsulate the scene,
// // the request and the response. We call the planner using this
// // planning context
// planning_interface::PlanningContextPtr context =
//     planner_instance->getPlanningContext(planning_scene, req, res.error_code_);

// if (!context)
// {
//   RCLCPP_ERROR(LOGGER, "Failed to create planning context");
//   return -1;
// }
// context->solve(res);
// if (res.error_code_.val != res.error_code_.SUCCESS)
// {
//   RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
//   return -1;
// }

// // Visualize the result
// // ^^^^^^^^^^^^^^^^^^^^
// std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
//     motion_planning_api_tutorial_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
//                                                                                              1);
// moveit_msgs::msg::DisplayTrajectory display_trajectory;

// /* Visualize the trajectory */
// moveit_msgs::msg::MotionPlanResponse response;
// res.getMessage(response);

// display_trajectory.trajectory_start = response.trajectory_start;
// display_trajectory.trajectory.push_back(response.trajectory);
// visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
// visual_tools.trigger();
// display_publisher->publish(display_trajectory);

// /* Set the state in the planning scene to the final state of the last plan */
// robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
// planning_scene->setCurrentState(*robot_state.get());

// // Display the goal state
// visual_tools.publishAxisLabeled(pose.pose, "goal_1");
// visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
// visual_tools.trigger();

// /* We can also use visual_tools to wait for user input */
// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

// // Joint Space Goals
// // ^^^^^^^^^^^^^^^^^
// // Now, setup a joint space goal
// moveit::core::RobotState goal_state(robot_model);
// std::vector<double> joint_values = { 0.3, 0.3, 0.3, 0.2, 0.2, 1.0};
// goal_state.setJointGroupPositions(joint_model_group, joint_values);
// moveit_msgs::msg::Constraints joint_goal =
//     kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
// req.goal_constraints.clear();
// req.goal_constraints.push_back(joint_goal);

// // Call the planner and visualize the trajectory
// /* Re-construct the planning context */
// context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
// /* Call the Planner */
// context->solve(res);
// /* Check that the planning was successful */
// if (res.error_code_.val != res.error_code_.SUCCESS)
// {
//   RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
//   return -1;
// }
// /* Visualize the trajectory */
// res.getMessage(response);
// display_trajectory.trajectory.push_back(response.trajectory);

// /* Now you should see two planned trajectories in series*/
// visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
// visual_tools.trigger();
// display_publisher->publish(display_trajectory);

/* We will add more goals. But first, set the state in the planning
   scene to the final state of the last plan */
// robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
// planning_scene->setCurrentState(*robot_state.get());

// // Display the goal state
// visual_tools.publishAxisLabeled(pose.pose, "goal_2");
// visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
// visual_tools.trigger();

// /* Wait for user input */
// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

// /* Now, we go back to the first goal to prepare for orientation constrained planning */
// req.goal_constraints.clear();
// req.goal_constraints.push_back(pose_goal);
// context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
// context->solve(res);
// res.getMessage(response);

// display_trajectory.trajectory.push_back(response.trajectory);
// visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
// visual_tools.trigger();
// display_publisher->publish(display_trajectory);

// /* Set the state in the planning scene to the final state of the last plan */
// robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
// planning_scene->setCurrentState(*robot_state.get());

// // Display the goal state
// visual_tools.trigger();

// /* Wait for user input */
// visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

// // Adding Path Constraints
// // ^^^^^^^^^^^^^^^^^^^^^^^
// // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
// /* Let's create a new pose goal */

// pose.pose.position.x = 0.32;
// pose.pose.position.y = -0.25;
// pose.pose.position.z = 0.65;
// pose.pose.orientation.w = 1.0;
// moveit_msgs::msg::Constraints pose_goal_2 =
//     kinematic_constraints::constructGoalConstraints("tool0", pose, tolerance_pose, tolerance_angle);

// /* Now, let's try to move to this new pose goal*/
// req.goal_constraints.clear();
// req.goal_constraints.push_back(pose_goal_2);

// /* But, let's impose a path constraint on the motion.
//    Here, we are asking for the end-effector to stay level*/
// geometry_msgs::msg::QuaternionStamped quaternion;
// quaternion.header.frame_id = "shoulder_link";
// req.path_constraints = kinematic_constraints::constructGoalConstraints("tool0", quaternion);

// // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
// // (the workspace of the robot)
// // because of this, we need to specify a bound for the allowed planning volume as well;
// // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
// // but that is not being used in this example).
// // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
// // in this volume
// // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
// req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
//     req.workspace_parameters.min_corner.z = -2.0;
// req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
//     req.workspace_parameters.max_corner.z = 2.0;

// // Call the planner and visualize all the plans created so far.
// context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
// context->solve(res);
// res.getMessage(response);
// display_trajectory.trajectory.push_back(response.trajectory);
// visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
// visual_tools.trigger();
// display_publisher->publish(display_trajectory);

// /* Set the state in the planning scene to the final state of the last plan */
// robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
// planning_scene->setCurrentState(*robot_state.get());

// // Display the goal state
// visual_tools.publishAxisLabeled(pose.pose, "goal_3");
// visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
// visual_tools.trigger();

// END_TUTORIAL
//   /* Wait for user input */
//   visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
//   planner_instance.reset();

//   rclcpp::shutdown();
//   return 0;
// }
