/**
 * @file detector_node.cpp
 * @brief Main ROS2 node for STag marker detection
 */

#include "stag_ros2_enhanced/detector_node.hpp"
#include <opencv2/calib3d.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace stag_ros2 {

STagDetectorNode::STagDetectorNode(const rclcpp::NodeOptions& options)
    : Node("stag_detector", options)
{
    declareAndLoadParameters();
    
    // Create detector based on marker type
    try {
        detector_ = createDetector(marker_type_, error_correction_, proximity_threshold_);
        RCLCPP_INFO(get_logger(), "Created %s detector (HD=%d, EC=%d, ProxThresh=%.1f)",
                    marker_type_.c_str(),
                    detector_->getHammingDistance(),
                    detector_->getErrorCorrection(),
                    proximity_threshold_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to create detector: %s", e.what());
        throw;
    }
    
    // Create marker tracker if enabled
    if (enable_marker_tracking_) {
        tracker_ = std::make_unique<MarkerTracker>(
            tracking_distance_threshold_,
            tracking_timeout_
        );
        RCLCPP_INFO(get_logger(), "Marker tracking enabled (threshold=%.1f px, timeout=%.2f s)",
                    tracking_distance_threshold_, tracking_timeout_);
    }
    
    // Create TF broadcaster if enabled
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        RCLCPP_INFO(get_logger(), "TF broadcasting enabled");
    }
    
    // Set up QoS profiles for image topics
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_,
        qos,
        std::bind(&STagDetectorNode::imageCallback, this, std::placeholders::_1)
    );
    
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        10,
        std::bind(&STagDetectorNode::cameraInfoCallback, this, std::placeholders::_1)
    );
    
    if (show_detection_) {
        detection_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            detection_image_topic_,
            10
        );
    }
    
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers",
        10
    );
    
    if (publish_statistics_) {
        stats_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "~/statistics",
            10
        );
    }
    
    // Initialize statistics
    last_stats_time_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(get_logger(), "STag Detector Node initialized");
    RCLCPP_INFO(get_logger(), "  Marker type: %s", marker_type_.c_str());
    RCLCPP_INFO(get_logger(), "  Image topic: %s", image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Show detection: %s", show_detection_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  Publish TF: %s", publish_tf_ ? "true" : "false");
    if (marker_size_ > 0 && publish_tf_) {
        RCLCPP_INFO(get_logger(), "  Marker size: %.3f m", marker_size_);
    }
}

void STagDetectorNode::declareAndLoadParameters()
{
    // Declare parameters with default values as fallback
    // These will be overridden by YAML if provided, or by launch file parameters
    this->declare_parameter("image_topic", "/camera/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/camera_info");
    this->declare_parameter("marker_type", "HD17");
    this->declare_parameter("error_correction", -1);
    this->declare_parameter("proximity_threshold", 10.0);
    this->declare_parameter("marker_size", 0.0);
    this->declare_parameter("show_detection", false);
    this->declare_parameter("detection_image_topic", "~/detection_image");
    this->declare_parameter("show_rejected", false);
    this->declare_parameter("publish_tf", true);
    this->declare_parameter("camera_frame", "camera_color_optical_frame");
    this->declare_parameter("marker_frame_prefix", "marker_");
    this->declare_parameter("enable_marker_tracking", true);
    this->declare_parameter("tracking_distance_threshold", 0.05);
    this->declare_parameter("tracking_timeout", 1.0);
    this->declare_parameter("publish_statistics", true);
    this->declare_parameter("statistics_update_rate", 1.0);
    
    // Get parameters (will use YAML values if provided, otherwise defaults above)
    image_topic_ = this->get_parameter("image_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    
    // Detection parameters
    marker_type_ = this->get_parameter("marker_type").as_string();
    error_correction_ = this->get_parameter("error_correction").as_int();
    proximity_threshold_ = this->get_parameter("proximity_threshold").as_double();
    
    // Marker physical properties
    marker_size_ = this->get_parameter("marker_size").as_double();
    
    // Visualization
    show_detection_ = this->get_parameter("show_detection").as_bool();
    detection_image_topic_ = this->get_parameter("detection_image_topic").as_string();
    show_rejected_ = this->get_parameter("show_rejected").as_bool();
    
    // TF broadcasting
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    marker_frame_prefix_ = this->get_parameter("marker_frame_prefix").as_string();
    
    // Marker tracking
    enable_marker_tracking_ = this->get_parameter("enable_marker_tracking").as_bool();
    tracking_distance_threshold_ = this->get_parameter("tracking_distance_threshold").as_double();
    tracking_timeout_ = this->get_parameter("tracking_timeout").as_double();
    
    // Statistics
    publish_statistics_ = this->get_parameter("publish_statistics").as_bool();
    statistics_update_rate_ = this->get_parameter("statistics_update_rate").as_double();
}

void STagDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (camera_info_received_) {
        return;
    }
    
    // Use P matrix (3x4) for rectified image — take upper-left 3x3 block
    // P layout: [ fx'  0   cx'  Tx | 0  fy'  cy'  Ty | 0  0  1  0 ]
    camera_matrix_ = cv::Mat(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->p[0];   // fx
    camera_matrix_.at<double>(0, 1) = msg->p[1];   // 0
    camera_matrix_.at<double>(0, 2) = msg->p[2];   // cx
    camera_matrix_.at<double>(1, 0) = msg->p[4];   // 0
    camera_matrix_.at<double>(1, 1) = msg->p[5];   // fy
    camera_matrix_.at<double>(1, 2) = msg->p[6];   // cy
    camera_matrix_.at<double>(2, 0) = msg->p[8];   // 0
    camera_matrix_.at<double>(2, 1) = msg->p[9];   // 0
    camera_matrix_.at<double>(2, 2) = msg->p[10];  // 1

    // Rectified image has zero distortion
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    camera_info_received_ = true;
    RCLCPP_INFO(get_logger(), "Camera calibration received (using P matrix, zero distortion for rectified image)");
}

void STagDetectorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto start_time = std::chrono::steady_clock::now();
    
    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    
    // Validate image
    if (cv_ptr->image.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received empty image, skipping frame");
        return;
    }
    
    // Detect markers
    auto detection_start = std::chrono::steady_clock::now();
    DetectionResult result;
    
    try {
        result = detector_->detect(cv_ptr->image);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Detection failed: %s", e.what());
        return;
    }
    
    auto detection_end = std::chrono::steady_clock::now();
    double detection_time = std::chrono::duration<double>(detection_end - detection_start).count();
    
    // Apply marker tracking if enabled
    if (enable_marker_tracking_ && tracker_ && !result.empty()) {
        result = tracker_->update(result);
    }
    
    // Publish RViz markers
    publishRVizMarkers(result, msg->header);
    
    // Publish detection image if enabled
    if (show_detection_ && detection_image_pub_) {
        try {
            publishDetectionImage(cv_ptr->image, result, msg->header);
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "OpenCV error in publishDetectionImage: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error in publishDetectionImage: %s", e.what());
        }
    }
    
    // Publish TF transforms if enabled
    if (publish_tf_ && camera_info_received_ && !result.empty() && marker_size_ > 0) {
        publishTFTransforms(result, msg->header);
    }
    
    // Update statistics
    auto end_time = std::chrono::steady_clock::now();
    double total_time = std::chrono::duration<double>(end_time - start_time).count();
    updateStatistics(total_time, result.size());
}

void STagDetectorNode::publishRVizMarkers(
    const DetectionResult& result,
    const std_msgs::msg::Header& header)
{
    auto marker_array = visualization_msgs::msg::MarkerArray();
    
    if (result.empty()) {
        // Publish empty array to clear old markers
        markers_pub_->publish(marker_array);
        return;
    }
    
    for (size_t i = 0; i < result.corners.size() && i < result.ids.size(); ++i) {
        int marker_id = result.ids[i];
        
        // Create line strip marker for border
        auto marker = visualization_msgs::msg::Marker();
        marker.header = header;
        marker.ns = "stag_markers";
        marker.id = marker_id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2.0;  // Line width
        
        std_msgs::msg::ColorRGBA color;
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 1.0;
        marker.color = color;
        
        // Add corner points
        for (const auto& point : result.corners[i]) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        
        // Close the loop
        geometry_msgs::msg::Point p;
        p.x = result.corners[i][0].x;
        p.y = result.corners[i][0].y;
        p.z = 0.0;
        marker.points.push_back(p);
        
        marker_array.markers.push_back(marker);
        
        // Create text marker for ID
        auto text_marker = visualization_msgs::msg::Marker();
        text_marker.header = header;
        text_marker.ns = "stag_marker_ids";
        text_marker.id = marker_id + 10000;  // Offset to avoid collision
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Calculate center
        cv::Point2f center(0, 0);
        for (const auto& point : result.corners[i]) {
            center += point;
        }
        center.x /= result.corners[i].size();
        center.y /= result.corners[i].size();
        
        text_marker.pose.position.x = center.x;
        text_marker.pose.position.y = center.y;
        text_marker.pose.position.z = 0.0;
        text_marker.pose.orientation.w = 1.0;
        
        text_marker.scale.z = 20.0;  // Text size
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        
        text_marker.text = "ID:" + std::to_string(marker_id);
        if (marker_size_ > 0) {
            text_marker.text += "\n" + std::to_string(static_cast<int>(marker_size_ * 1000)) + "mm";
        }
        
        marker_array.markers.push_back(text_marker);
    }
    
    markers_pub_->publish(marker_array);
}

void STagDetectorNode::addDetectionOverlay(
    cv::Mat& image,
    const DetectionResult& result,
    double fps)
{
    // Check if image is valid
    if (image.empty()) {
        return;
    }
    
    int h = image.rows;
    int w = image.cols;
    
    // Draw semi-transparent background in bottom-left
    cv::Mat overlay = image.clone();
    cv::rectangle(
        overlay,
        cv::Point(5, h - 100),
        cv::Point(350, h - 5),
        cv::Scalar(0, 0, 0),
        cv::FILLED
    );
    cv::addWeighted(overlay, 0.6, image, 0.4, 0, image);
    
    // Add detection count
    std::string det_text = "Detected: " + std::to_string(result.size()) +
                           " | Rejected: " + std::to_string(result.rejected.size());
    cv::putText(
        image,
        det_text,
        cv::Point(10, h - 70),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 0),
        2
    );
    
    // Add FPS
    std::string fps_text = "FPS: " + std::to_string(static_cast<int>(fps));
    cv::putText(
        image,
        fps_text,
        cv::Point(10, h - 40),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 0),
        2
    );
    
    // Add marker type
    std::string type_text = "Marker Type: " + marker_type_;
    cv::putText(
        image,
        type_text,
        cv::Point(10, h - 10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 255, 0),
        2
    );
}

void STagDetectorNode::publishDetectionImage(
    const cv::Mat& image,
    const DetectionResult& result,
    const std_msgs::msg::Header& header)
{
    // Check if image is valid
    if (image.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Cannot publish detection image: image is empty");
        return;
    }
    
    cv::Mat detection_image = image.clone();
    
    // Draw detections
    try {
        utils::drawDetections(detection_image, result, show_rejected_, marker_type_);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV error in drawDetections: %s", e.what());
        return;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in drawDetections: %s", e.what());
        return;
    }
    
    // Calculate FPS
    double fps = 0.0;
    if (!detection_times_.empty()) {
        size_t count = std::min(detection_times_.size(), size_t(30));
        double sum = 0.0;
        for (size_t i = detection_times_.size() - count; i < detection_times_.size(); ++i) {
            sum += detection_times_[i];
        }
        double avg_time = sum / count;
        fps = (avg_time > 0) ? 1.0 / avg_time : 0.0;
    }
    
    // Add overlay
    try {
        addDetectionOverlay(detection_image, result, fps);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV error in addDetectionOverlay: %s", e.what());
        return;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error in addDetectionOverlay: %s", e.what());
        return;
    }
    
    // Convert back to ROS message
    auto msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, detection_image).toImageMsg();
    detection_image_pub_->publish(*msg);
}

void STagDetectorNode::publishTFTransforms(
    const DetectionResult& result,
    const std_msgs::msg::Header& header)
{
    for (size_t i = 0; i < result.corners.size() && i < result.ids.size(); ++i) {
        int marker_id = result.ids[i];
        
        // Estimate pose
        auto pose_opt = PoseEstimator::estimateMarkerPose(
            result.corners[i],
            marker_size_,
            camera_matrix_,
            dist_coeffs_
        );
        
        if (!pose_opt.has_value()) {
            RCLCPP_DEBUG(get_logger(), "Failed to estimate pose for marker %d", marker_id);
            continue;
        }

        // solvePnP returns pose in OpenCV optical frame (Z forward, X right, Y down).
        // TF parent is camera_frame_, a body frame in REP-103 convention (X forward, Y left, Z up;
        // sim: torso_rgbd), so the optical->body rotation R_cl_from_opt = [0 0 1; -1 0 0; 0 -1 0] is applied.
        const Pose& pose_opt_frame = pose_opt.value();
        cv::Mat R_cl_from_opt = (cv::Mat_<double>(3, 3) <<
             0,  0,  1,
            -1,  0,  0,
             0, -1,  0);

        cv::Mat t_opt = (cv::Mat_<double>(3, 1) <<
            pose_opt_frame.translation[0],
            pose_opt_frame.translation[1],
            pose_opt_frame.translation[2]);
        cv::Mat t_cl = R_cl_from_opt * t_opt;

        cv::Mat R_opt_marker;
        cv::Rodrigues(pose_opt_frame.rotation, R_opt_marker);
        cv::Mat R_cl_marker = R_cl_from_opt * R_opt_marker;
        cv::Vec3d rvec_cl;
        cv::Rodrigues(R_cl_marker, rvec_cl);

        Pose pose_cl(
            cv::Vec3d(t_cl.at<double>(0), t_cl.at<double>(1), t_cl.at<double>(2)),
            rvec_cl
        );

        // Create and broadcast transform
        auto transform = PoseEstimator::createTransformStamped(
            pose_cl,
            marker_id,
            camera_frame_,
            marker_frame_prefix_,
            rclcpp::Time(header.stamp)
        );

        tf_broadcaster_->sendTransform(transform);
    }
}

void STagDetectorNode::updateStatistics(double total_time, size_t num_detected)
{
    detection_times_.push_back(total_time);
    if (detection_times_.size() > 100) {
        detection_times_.erase(detection_times_.begin());
    }
    
    frame_count_++;
    
    // Log statistics at specified rate (additionally throttled to 5 s)
    if (publish_statistics_) {
        auto current_time = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(current_time - last_stats_time_).count();
        
        if (elapsed >= 1.0 / statistics_update_rate_) {
            double sum = 0.0;
            for (double t : detection_times_) {
                sum += t;
            }
            double avg_time = sum / detection_times_.size();
            double fps = (avg_time > 0) ? 1.0 / avg_time : 0.0;
            
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,  // 5 seconds
                "Stats: FPS=%.1f, Avg time=%.1f ms, Frames=%zu",
                fps, avg_time * 1000.0, frame_count_
            );
            
            last_stats_time_ = current_time;
        }
    }
}

} // namespace stag_ros2
