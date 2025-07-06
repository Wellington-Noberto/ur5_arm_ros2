#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2/exceptions.h"

class AprilTagTFSubscriber : public rclcpp::Node
{
public:
    AprilTagTFSubscriber()
        : Node("apriltag_tf_subscriber"),
          tf_broadcaster_(this),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/world/empty/pose/info", 10,
            std::bind(&AprilTagTFSubscriber::pose_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AprilTagTFSubscriber::publish_and_lookup_tf, this));
    }

private:
    void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& tf : msg->transforms)
        {
            if (tf.child_frame_id == "apriltag_0")  // Only process the apriltag_0 frame
            {
                last_pose_ = tf;  // Store the transform stamped
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
        t.header.frame_id = "world";  // Adjust if your apriltag_0 is relative to another frame
        t.child_frame_id = "apriltag_0";

        tf_broadcaster_.sendTransform(t);

        // Now try to lookup transform from camera_ros_link to apriltag_0
        try
        {
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
                "camera_ros_link", "world", tf2::TimePointZero);

            RCLCPP_INFO(this->get_logger(),
                "Transform from camera_ros_link to apriltag_0: translation [%.3f, %.3f, %.3f]",
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z);
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagTFSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
