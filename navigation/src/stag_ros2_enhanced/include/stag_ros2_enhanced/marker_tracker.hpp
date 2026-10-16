#ifndef STAG_ROS2_ENHANCED__MARKER_TRACKER_HPP_
#define STAG_ROS2_ENHANCED__MARKER_TRACKER_HPP_

#include <unordered_map>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "stag_ros2_enhanced/detection_strategies.hpp"

namespace stag_ros2 {

/**
 * @brief Tracks markers over time for stable detections
 */
class MarkerTracker {
public:
    /**
     * @brief Constructor
     * @param distance_threshold Maximum distance to consider markers as same (pixels)
     * @param timeout_seconds Time after which a marker is forgotten (seconds)
     */
    explicit MarkerTracker(
        double distance_threshold = 10.0,
        double timeout_seconds = 0.5
    );
    
    /**
     * @brief Update tracker with new detections
     * @param result Current frame detection result
     * @return Filtered and tracked detection result
     */
    DetectionResult update(const DetectionResult& result);
    
    /**
     * @brief Clear all tracking history
     */
    void clear();
    
    /**
     * @brief Get number of currently tracked markers
     */
    size_t getTrackedCount() const { return last_detection_time_.size(); }

private:
    double distance_threshold_;
    std::chrono::duration<double> timeout_;
    
    // Track last detection time for each marker ID
    std::unordered_map<int, std::chrono::steady_clock::time_point> last_detection_time_;
    
    // Track last known position for each marker ID
    std::unordered_map<int, cv::Point2f> last_positions_;
    
    /**
     * @brief Remove old markers from tracking
     */
    void removeOldMarkers();
    
    /**
     * @brief Calculate center of marker corners
     */
    static cv::Point2f calculateCenter(const std::vector<cv::Point2f>& corners);
};

} // namespace stag_ros2

#endif  // STAG_ROS2_ENHANCED__MARKER_TRACKER_HPP_
