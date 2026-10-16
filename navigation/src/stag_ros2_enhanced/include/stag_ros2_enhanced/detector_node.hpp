#ifndef STAG_ROS2_ENHANCED__DETECTOR_NODE_HPP_
#define STAG_ROS2_ENHANCED__DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.hpp>

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "stag_ros2_enhanced/detection_strategies.hpp"
#include "stag_ros2_enhanced/pose_estimation.hpp"
#include "stag_ros2_enhanced/marker_tracker.hpp"

namespace stag_ros2 {

/**
 * @brief Main ROS2 node for STag marker detection
 * 
 * This node subscribes to camera image and camera_info topics,
 * detects STag markers, and publishes:
 * - Detection visualization images
 * - RViz markers for visualization
 * - TF transforms for detected markers (optional)
 */
class STagDetectorNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor with node options
     * @param options Node options for composition
     */
    explicit STagDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    ~STagDetectorNode() override = default;

private:
    // === Callbacks ===
    
    /**
     * @brief Process incoming camera images
     */
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    /**
     * @brief Process camera calibration info
     */
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    
    // === Publishers and Subscribers ===
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr detection_image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stats_pub_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // === Core Components ===
    
    std::unique_ptr<STagDetectionStrategy> detector_;
    std::unique_ptr<MarkerTracker> tracker_;
    
    // === Camera Calibration ===
    
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_{false};
    
    // === Parameters ===
    
    // Image topics
    std::string image_topic_;
    std::string camera_info_topic_;
    
    // Detection parameters
    std::string marker_type_;
    int error_correction_;
    double proximity_threshold_;  // For HC marker centroid matching (pixels)
    
    // Marker physical properties
    double marker_size_;  // meters
    
    // Visualization
    bool show_detection_;
    std::string detection_image_topic_;
    bool show_rejected_;
    
    // TF broadcasting
    bool publish_tf_;
    std::string camera_frame_;
    std::string marker_frame_prefix_;
    
    // Marker tracking
    bool enable_marker_tracking_;
    double tracking_distance_threshold_;
    double tracking_timeout_;
    
    // Statistics
    bool publish_statistics_;
    double statistics_update_rate_;
    
    // === Statistics ===
    
    std::vector<double> detection_times_;
    size_t frame_count_{0};
    std::chrono::steady_clock::time_point last_stats_time_;
    
    // === Helper Methods ===
    
    /**
     * @brief Declare and load all ROS2 parameters
     */
    void declareAndLoadParameters();
    
    /**
     * @brief Publish RViz visualization markers
     */
    void publishRVizMarkers(
        const DetectionResult& result,
        const std_msgs::msg::Header& header
    );
    
    /**
     * @brief Publish detection image with overlays
     */
    void publishDetectionImage(
        const cv::Mat& image,
        const DetectionResult& result,
        const std_msgs::msg::Header& header
    );
    
    /**
     * @brief Publish TF transforms for detected markers
     */
    void publishTFTransforms(
        const DetectionResult& result,
        const std_msgs::msg::Header& header
    );
    
    /**
     * @brief Update and optionally publish statistics
     */
    void updateStatistics(double detection_time, size_t num_detected);
    
    /**
     * @brief Add overlay text to detection image
     */
    void addDetectionOverlay(
        cv::Mat& image,
        const DetectionResult& result,
        double fps
    );
};

} // namespace stag_ros2

#endif  // STAG_ROS2_ENHANCED__DETECTOR_NODE_HPP_
