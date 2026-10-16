#ifndef STAG_ROS2_ENHANCED__DETECTION_LOGGER_NODE_HPP_
#define STAG_ROS2_ENHANCED__DETECTION_LOGGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <fstream>
#include <string>
#include <vector>

namespace stag_ros2 {

/**
 * @brief ROS2 node that logs marker detections to a JSONL file
 * 
 * Subscribes to MarkerArray topic and writes detection data in JSON Lines format:
 * {"timestamp": 1729692345.123456, "frame_id": 0, "marker_ids": [0, 5, 12]}
 */
class DetectionLoggerNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor with node options
     * @param options Node options for composition
     */
    explicit DetectionLoggerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    
    ~DetectionLoggerNode() override;

private:
    /**
     * @brief Callback for incoming MarkerArray messages
     */
    void markersCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    
    /**
     * @brief Declare and load all ROS2 parameters
     */
    void declareAndLoadParameters();
    
    /**
     * @brief Setup log file for writing
     */
    void setupLogFile();
    
    /**
     * @brief Write log entry in JSONL format
     */
    void writeLogEntry(double timestamp, size_t frame_id, const std::vector<int>& marker_ids);
    
    /**
     * @brief Write header to log file
     */
    void writeHeader();
    
    /**
     * @brief Write footer to log file
     */
    void writeFooter();
    
    // Subscription
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
    
    // Log file
    std::ofstream log_file_;
    std::string log_file_path_;
    
    // Parameters
    std::string markers_topic_;
    bool include_metadata_;
    bool auto_create_dirs_;
    
    // Statistics
    size_t frame_count_{0};
};

} // namespace stag_ros2

#endif  // STAG_ROS2_ENHANCED__DETECTION_LOGGER_NODE_HPP_
