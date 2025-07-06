#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag_pose.h>
#include <apriltag/tag36h11.h>
#include <eigen3/Eigen/Eigen>
#include "tf2_ros/transform_broadcaster.h"


#include <sstream>
#include <memory>

class AprilTagDetectorNode : public rclcpp::Node {
public:
AprilTagDetectorNode() :
        Node("marker_detector"),
        tf_broadcaster_(this)
     {
        // Image subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera", 10,
            std::bind(&AprilTagDetectorNode::image_callback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("img_markers", 10);

        // Detection publisher
        detection_pub_ = this->create_publisher<std_msgs::msg::String>("apriltag_detections", 10);

        this->declare_parameter<std::string>("frame_id", "camera_frame");
        this->declare_parameter<double>("tag_size", 0.1);
        this->declare_parameter<double>("fx", 0.0);
        this->declare_parameter<double>("fy", 0.0);
        this->declare_parameter<double>("cx", 0.0);
        this->declare_parameter<double>("cy", 0.0);

        this->get_parameter("frame_id", frame_id_);
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

    ~AprilTagDetectorNode() {
        apriltag_detector_destroy(td_);
        tag36h11_destroy(tf_);
    }

private:
    void publish_debug_image(const cv::Mat& image, const std::string& frame_id = "camera_frame")
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

            // Print 3D pose
            std::cout << "Translation (x,y,z): "
            << pose.t->data[0] << ", "
            << pose.t->data[1] << ", "
            << pose.t->data[2] << std::endl;

            std::cout << "Rotation matrix:" << std::endl;
            for (int i = 0; i < 3; ++i) {
                std::cout << pose.R->data[i * 3 + 0] << " "
                << pose.R->data[i * 3 + 1] << " "
                << pose.R->data[i * 3 + 2] << std::endl;
            }

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
        transform.header.frame_id = frame_id_;  // Change if your base frame differs
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

        // Publish
        tf_broadcaster_.sendTransform(transform);
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

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detection_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    apriltag_family_t* tf_;
    apriltag_detector_t* td_;
    std::string frame_id_;
    double tag_size_;
    double fx_, fy_, cx_, cy_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AprilTagDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
