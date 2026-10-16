/**
 * @file marker_tracker.cpp
 * @brief Marker tracking for stable detections over time
 */

#include "stag_ros2_enhanced/marker_tracker.hpp"
#include <cmath>

namespace stag_ros2 {

MarkerTracker::MarkerTracker(double distance_threshold, double timeout_seconds)
    : distance_threshold_(distance_threshold),
      timeout_(timeout_seconds)
{
}

void MarkerTracker::clear()
{
    last_detection_time_.clear();
    last_positions_.clear();
}

cv::Point2f MarkerTracker::calculateCenter(const std::vector<cv::Point2f>& corners)
{
    cv::Point2f center(0, 0);
    for (const auto& point : corners) {
        center += point;
    }
    center.x /= corners.size();
    center.y /= corners.size();
    return center;
}

void MarkerTracker::removeOldMarkers()
{
    auto current_time = std::chrono::steady_clock::now();
    
    // Remove markers that haven't been seen within timeout
    for (auto it = last_detection_time_.begin(); it != last_detection_time_.end();) {
        auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
            current_time - it->second
        );
        
        if (elapsed > timeout_) {
            last_positions_.erase(it->first);
            it = last_detection_time_.erase(it);
        } else {
            ++it;
        }
    }
}

DetectionResult MarkerTracker::update(const DetectionResult& result)
{
    removeOldMarkers();
    
    if (result.empty()) {
        return result;
    }
    
    auto current_time = std::chrono::steady_clock::now();
    DetectionResult filtered = result;
    
    // Filter duplicates based on distance threshold
    std::vector<bool> keep(result.corners.size(), true);
    
    for (size_t i = 0; i < result.corners.size(); ++i) {
        if (!keep[i]) continue;
        
        cv::Point2f center_i = calculateCenter(result.corners[i]);
        
        for (size_t j = i + 1; j < result.corners.size(); ++j) {
            if (!keep[j]) continue;
            
            cv::Point2f center_j = calculateCenter(result.corners[j]);
            
            // Calculate distance
            double dx = center_i.x - center_j.x;
            double dy = center_i.y - center_j.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            // If too close, mark as duplicate
            if (dist < distance_threshold_) {
                keep[j] = false;
            }
        }
    }
    
    // Build filtered result
    filtered.clear();
    for (size_t i = 0; i < result.corners.size(); ++i) {
        if (keep[i]) {
            filtered.corners.push_back(result.corners[i]);
            if (i < result.ids.size()) {
                filtered.ids.push_back(result.ids[i]);
                
                // Update tracking info
                int marker_id = result.ids[i];
                cv::Point2f center = calculateCenter(result.corners[i]);
                
                last_detection_time_[marker_id] = current_time;
                last_positions_[marker_id] = center;
            }
        }
    }
    
    // Keep all rejected
    filtered.rejected = result.rejected;
    
    return filtered;
}

} // namespace stag_ros2
