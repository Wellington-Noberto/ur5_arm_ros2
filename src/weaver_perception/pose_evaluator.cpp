#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/exceptions.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

class PoseEvaluator : public rclcpp::Node
{
public:
    PoseEvaluator()
        : Node("pose_evaluator"),
          tf_broadcaster_(this),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/world/empty/pose/info", 10,
            std::bind(&PoseEvaluator::pose_callback, this, std::placeholders::_1));

        this->declare_parameter<double>("x", 1.9);
        this->declare_parameter<double>("y", -2.0);
        this->declare_parameter<double>("z", 0.85);
        this->declare_parameter<double>("roll", 0.0);
        this->declare_parameter<double>("pitch", 0.0);
        this->declare_parameter<double>("yaw", 0.0);

        this->get_parameter("x", spawn_x);
        this->get_parameter("y", spawn_y);
        this->get_parameter("z", spawn_z);
        this->get_parameter("roll", spawn_roll);
        this->get_parameter("pitch", spawn_pitch);
        this->get_parameter("yaw", spawn_yaw);

        spawn_pose = Eigen::Affine3d::Identity();
        spawn_pose.translate(Eigen::Vector3d(spawn_x, spawn_y, spawn_z));
        spawn_pose.rotate(Eigen::AngleAxisd(spawn_yaw, Eigen::Vector3d::UnitZ()));
        spawn_pose.rotate(Eigen::AngleAxisd(spawn_pitch, Eigen::Vector3d::UnitY()));
        spawn_pose.rotate(Eigen::AngleAxisd(spawn_roll, Eigen::Vector3d::UnitX()));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PoseEvaluator::publish_and_lookup_tf, this));
    }

private:
    void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto &tf : msg->transforms)
        {
            if (tf.child_frame_id == "apriltag_0") // Only process the apriltag_0 frame
            {
                last_pose_ = tf; // Store the transform stamped
                got_pose_ = true;
            }
        }
    }

    void publish_and_lookup_tf()
    {
        if (!got_pose_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Waiting for apriltag_0 pose...");
            return;
        }

        // Publish the apriltag_0 transform relative to "world"
        geometry_msgs::msg::TransformStamped t = last_pose_;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world_gazebo"; // Adjust if your apriltag_0 is relative to another frame
        t.child_frame_id = "apriltag_0";

        tf_broadcaster_.sendTransform(t);

        // Now try to lookup transform from camera_ros_link to apriltag_0
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
                "world", "camera_ros_link", tf2::TimePointZero);

            Eigen::Affine3d tf_camera_world = tf2::transformToEigen(transformStamped).inverse();
            Eigen::Affine3d tf_world_apriltag = tf2::transformToEigen(t);
            Eigen::Affine3d tf_camera_apriltag = tf_camera_world * spawn_pose.inverse() * tf_world_apriltag;

            geometry_msgs::msg::TransformStamped t_final = tf2::eigenToTransform(tf_camera_apriltag);
            geometry_msgs::msg::TransformStamped t_camera_world = tf2::eigenToTransform(tf_camera_world);

            RCLCPP_INFO(this->get_logger(),
                        "Transform from camera_ros_link to apriltag_0: translation [%.3f, %.3f, %.3f]",
                        t_final.transform.translation.x,
                        t_final.transform.translation.y,
                        t_final.transform.translation.z);

            RCLCPP_INFO(this->get_logger(),
                        "Transform from camera_ros_link to world: translation [%.3f, %.3f, %.3f]",
                        t_camera_world.transform.translation.x,
                        t_camera_world.transform.translation.y,
                        t_camera_world.transform.translation.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF lookup failed: %s", ex.what());
        }
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped last_pose_;
    bool got_pose_ = false;

    double spawn_x, spawn_y, spawn_z, spawn_roll, spawn_pitch, spawn_yaw;
    Eigen::Affine3d spawn_pose;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseEvaluator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
