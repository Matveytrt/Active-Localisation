/**
 * @file detection_logger_node.cpp
 * @brief Detection logger node implementation
 */

#include "stag_ros2_enhanced/detection_logger_node.hpp"
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <ctime>

namespace stag_ros2 {

DetectionLoggerNode::DetectionLoggerNode(const rclcpp::NodeOptions& options)
    : Node("detection_logger", options)
{
    // Declare and load parameters
    declareAndLoadParameters();
    
    // Setup log file
    setupLogFile();
    
    // Create subscription
    markers_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        markers_topic_,
        10,
        std::bind(&DetectionLoggerNode::markersCallback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(get_logger(), "Detection Logger initialized");
    RCLCPP_INFO(get_logger(), "  Logging to: %s", log_file_path_.c_str());
    RCLCPP_INFO(get_logger(), "  Markers topic: %s", markers_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  Include metadata: %s", include_metadata_ ? "true" : "false");
}

DetectionLoggerNode::~DetectionLoggerNode()
{
    if (log_file_.is_open()) {
        writeFooter();
        log_file_.close();
        RCLCPP_INFO(get_logger(), "Log file closed. Total frames logged: %zu", frame_count_);
    }
}

void DetectionLoggerNode::declareAndLoadParameters()
{
    // Log file path
    log_file_path_ = declare_parameter("log_file_path", "detections.jsonl");
    
    // Markers topic
    markers_topic_ = declare_parameter("markers_topic", "/stag_detector/markers");
    
    // Include metadata
    include_metadata_ = declare_parameter("include_metadata", false);
    
    // Auto-create directories
    auto_create_dirs_ = declare_parameter("auto_create_dirs", true);
}

void DetectionLoggerNode::setupLogFile()
{
    try {
        namespace fs = std::filesystem;
        
        // Expand path
        fs::path log_path(log_file_path_);
        
        // Handle ~ expansion
        if (!log_file_path_.empty() && log_file_path_[0] == '~') {
            const char* home = std::getenv("HOME");
            if (home) {
                log_path = fs::path(home) / log_file_path_.substr(2);
            }
        }
        
        // Create directories if needed
        if (auto_create_dirs_ && log_path.has_parent_path()) {
            fs::create_directories(log_path.parent_path());
        }
        
        // Check if file exists to determine if we should write header
        bool file_exists = fs::exists(log_path) && fs::file_size(log_path) > 0;
        
        // Open file for appending
        log_file_.open(log_path, std::ios::app);
        
        if (!log_file_.is_open()) {
            throw std::runtime_error("Failed to open log file: " + log_path.string());
        }
        
        // Write header if new file
        if (!file_exists) {
            writeHeader();
        }
        
        RCLCPP_INFO(get_logger(), "Log file opened: %s", log_path.string().c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to setup log file: %s", e.what());
        throw;
    }
}

void DetectionLoggerNode::writeHeader()
{
    // Get current time in ISO format
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
    
    // Write header comment (JSONL supports comments with //)
    log_file_ << "// {\"log_type\": \"stag_detection_log\", "
              << "\"version\": \"1.0\", "
              << "\"created_at\": \"" << ss.str() << "\", "
              << "\"format\": \"Each line is a JSON object with timestamp, frame_id, and marker_ids\"}\n";
    log_file_.flush();
}

void DetectionLoggerNode::writeFooter()
{
    // Get current time in ISO format
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t), "%Y-%m-%dT%H:%M:%SZ");
    
    log_file_ << "// {\"log_type\": \"stag_detection_log_end\", "
              << "\"total_frames\": " << frame_count_ << ", "
              << "\"closed_at\": \"" << ss.str() << "\"}\n";
    log_file_.flush();
}

void DetectionLoggerNode::writeLogEntry(
    double timestamp,
    size_t frame_id,
    const std::vector<int>& marker_ids)
{
    try {
        // Start JSON object
        log_file_ << "{\"timestamp\": " << std::fixed << std::setprecision(6) << timestamp
                  << ", \"frame_id\": " << frame_id
                  << ", \"marker_ids\": [";
        
        // Write marker IDs
        for (size_t i = 0; i < marker_ids.size(); ++i) {
            if (i > 0) log_file_ << ", ";
            log_file_ << marker_ids[i];
        }
        log_file_ << "]";
        
        // Add metadata if enabled
        if (include_metadata_) {
            log_file_ << ", \"num_markers\": " << marker_ids.size();
        }
        
        // Close JSON object
        log_file_ << "}\n";
        log_file_.flush();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to write log entry: %s", e.what());
    }
}

void DetectionLoggerNode::markersCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    // Extract marker IDs (filter out text markers which have different namespace)
    std::vector<int> marker_ids;
    
    for (const auto& marker : msg->markers) {
        // Only include markers from "stag_markers" namespace (not text markers)
        if (marker.ns == "stag_markers") {
            marker_ids.push_back(marker.id);
        }
    }
    
    // Sort for consistency
    std::sort(marker_ids.begin(), marker_ids.end());
    
    // Get current timestamp
    auto now = this->get_clock()->now();
    double timestamp_sec = now.seconds();
    
    // Write log entry
    writeLogEntry(timestamp_sec, frame_count_, marker_ids);
    
    frame_count_++;
    
    // Periodic logging
    if (frame_count_ % 100 == 0) {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,  // 5 seconds
            "Logged %zu frames, last detection: %zu markers",
            frame_count_,
            marker_ids.size()
        );
    }
}

} // namespace stag_ros2
