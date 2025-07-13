#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"


#include <sstream>
#include <memory>

class MarkerDetectorNode : public rclcpp::Node {
public:
MarkerDetectorNode() :
        Node("marker_detector"),
        tf_broadcaster_(this),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
{
        // Image subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&MarkerDetectorNode::image_callback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("img_markers", 10);

        // Detection publisher
        detection_pub_ = this->create_publisher<std_msgs::msg::String>("apriltag_detections", 10);

        this->declare_parameter<std::string>("camera_frame_id", "camera_ros_link");
        this->declare_parameter<std::string>("tool_frame_id", "tool0");
        this->declare_parameter<double>("tag_size", 0.1);
        this->declare_parameter<double>("fx", 0.0);
        this->declare_parameter<double>("fy", 0.0);
        this->declare_parameter<double>("cx", 0.0);
        this->declare_parameter<double>("cy", 0.0);

        this->get_parameter("camera_frame_id", camera_frame_id_);
        this->get_parameter("tool_frame_id", tool_frame_id_);
        this->get_parameter("tag_size", tag_size_);
        this->get_parameter("fx", fx_);
        this->get_parameter("fy", fy_);
        this->get_parameter("cx", cx_);
        this->get_parameter("cy", cy_);

        // Setup AprilTag detector
        tf_ = tag36h11_create();
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);

        RCLCPP_INFO(this->get_logger(), "AprilTag detector node started.");
    }

    ~MarkerDetectorNode() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    void publish_debug_image(const cv::Mat& image, const std::string& frame_id)
    {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = frame_id;

        // Convert OpenCV image (BGR) to ROS2 image message
        auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

        image_pub_->publish(*msg);
    }

    void pose_estimation(cv::Mat& img , zarray_t* detections)
    {
        if (img.type() != CV_8UC3) {
            RCLCPP_ERROR(this->get_logger(), "Input image is not CV_8UC3 (type=%d), skipping", img.type());
            return;
        }
        for (int i = 0; i < zarray_size(detections); ++i)
        {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = tag_size_;
            info.fx = fx_;
            info.fy = fy_;
            info.cx = cx_;
            info.cy = cy_;

            apriltag_pose_t pose;
            estimate_tag_pose(&info, &pose);

            publish_tag_tf(pose, det->id);

            cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
                fx_, 0, cx_,
                0, fy_, cy_,
                0, 0, 1);
            cv::Mat distortion_coefficients = cv::Mat::zeros(4, 1, CV_64F); // assume no distortion

            cv::Mat R(3, 3, CV_64F, pose.R->data);
            cv::Vec3d t(pose.t->data[0], pose.t->data[1], pose.t->data[2]);
            cv::Mat tvec(t);
            cv::Mat rvec;
            cv::Rodrigues(R, rvec);
            cv::drawFrameAxes(img, camera_matrix, distortion_coefficients, rvec, tvec, 0.2, 3);
        }
        publish_debug_image(img, "camera_frame");
    }

    void publish_tag_tf(const apriltag_pose_t& pose, int tag_id)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = camera_frame_id_;
        transform.child_frame_id = "apriltag_" + std::to_string(tag_id);

        // Translation
        transform.transform.translation.x = pose.t->data[0];
        transform.transform.translation.y = pose.t->data[1];
        transform.transform.translation.z = pose.t->data[2];

        // Convert rotation matrix to quaternion
        cv::Mat R(3, 3, CV_64F, pose.R->data);
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);

        cv::Mat quat;
        cv::Rodrigues(rvec, R);  // Reconvert just to make sure it's consistent
        Eigen::Matrix3d eigen_R;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                eigen_R(i, j) = R.at<double>(i, j);

        Eigen::Quaterniond q(eigen_R);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // Publish Camera to Apriltag transform
        tf_broadcaster_.sendTransform(transform);

        // Publish Tool to apriltag transform
        const Eigen::Quaterniond optical_to_camera =
            Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());


        // Get the transformation from camera to tool
        geometry_msgs::msg::TransformStamped tool_camera_ts = tf_buffer_.lookupTransform(
                tool_frame_id_, camera_frame_id_, tf2::TimePointZero);

        Eigen::Affine3d tool_apriltag_transformation = Eigen::Affine3d::Identity();
        Eigen::Affine3d tool_camera_transformation = tf2::transformToEigen(tool_camera_ts);
        Eigen::Affine3d camera_apriltag_transformation = tf2::transformToEigen(transform);

        camera_apriltag_transformation.prerotate(optical_to_camera);
        camera_apriltag_transformation.rotate(optical_to_camera);

        tool_apriltag_transformation = tool_camera_transformation * camera_apriltag_transformation;

        geometry_msgs::msg::TransformStamped tool_apriltag_msg = tf2::eigenToTransform(tool_apriltag_transformation);

        tool_apriltag_msg.header.stamp = this->get_clock()->now();
        tool_apriltag_msg.header.frame_id = tool_frame_id_;
        tool_apriltag_msg.child_frame_id = "apriltag_" + std::to_string(tag_id);

        // Publish Tool to Apriltag transform
        tf_broadcaster_.sendTransform(tool_apriltag_msg);

        RCLCPP_INFO(this->get_logger(),
                "Transform from Tool to Apriltag: translation [%.3f, %.3f, %.3f]",
                tool_apriltag_msg.transform.translation.x,
                tool_apriltag_msg.transform.translation.y,
                tool_apriltag_msg.transform.translation.z);

        Eigen::Vector3d euler_angles = tool_apriltag_transformation.rotation().eulerAngles(2, 1, 0);
        RCLCPP_INFO(this->get_logger(),
                "Transform from Tool to Apriltag: rotation \n yaw %.3f, \n pitch %.3f, \n roll %.3f",
                euler_angles[0],
                euler_angles[1],
                euler_angles[2]);
    }


    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert image to OpenCV BGR (color) image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");  // Assume input image is already color
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat& image = cv_ptr->image;

        // Prepare AprilTag input format (grayscale image required)
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        image_u8_t img_header = {gray.cols, gray.rows, gray.cols, gray.data};

        zarray_t* detections = apriltag_detector_detect(td_, &img_header);

        std::ostringstream ss;
        ss << "Detected " << zarray_size(detections) << " tags\n";

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            ss << "Tag ID: " << det->id << " center: ("
               << det->c[0] << ", " << det->c[1] << ")\n";
        }

        pose_estimation(image, detections);

        apriltag_detections_destroy(detections);
    }

    void tf_sub_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        for (const auto& tf_static : msg->transforms)
        {
            if (tf_static.child_frame_id == camera_frame_id_)  // Only process the apriltag_0 frame
            {
                last_pose_ = tf_static;  // Store the transform stamped
                got_pose_ = true;
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    apriltag_family_t* tf_;
    apriltag_detector_t* td_;
    std::string camera_frame_id_, tool_frame_id_;
    double tag_size_;
    double fx_, fy_, cx_, cy_;
    bool got_pose_ = false;

    geometry_msgs::msg::TransformStamped last_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MarkerDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
