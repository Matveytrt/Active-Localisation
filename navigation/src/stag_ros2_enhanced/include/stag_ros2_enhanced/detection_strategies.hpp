#ifndef STAG_ROS2_ENHANCED__DETECTION_STRATEGIES_HPP_
#define STAG_ROS2_ENHANCED__DETECTION_STRATEGIES_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <future>
#include "Stag.h"  // STag library header

namespace stag_ros2 {

/**
 * @brief Structure to hold detection results
 */
struct DetectionResult {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;
    
    void clear() {
        corners.clear();
        ids.clear();
        rejected.clear();
    }
    
    bool empty() const {
        return corners.empty();
    }
    
    size_t size() const {
        return corners.size();
    }
};

/**
 * @brief Structure to hold detection for a single channel (for HC markers)
 */
struct ChannelDetection {
    cv::Point2f centroid;
    std::vector<cv::Point2f> corners;
    int id;
    int channel_index;  // 0=B, 1=G, 2=R
};

/**
 * @brief Structure for matched triple detection (for HC markers)
 */
struct TripleMatch {
    ChannelDetection red;
    ChannelDetection green;
    ChannelDetection blue;
    bool has_red = false;
    bool has_green = false;
    bool has_blue = false;
    
    bool isComplete() const {
        return has_red && has_green && has_blue;
    }
    
    bool hasAny() const {
        return has_red || has_green || has_blue;
    }
    
    int computeHCId(int hamming_distance) const {
        // HC = IDr + IDg*|HD| + IDb*|HD|²
        // Use 0 for missing channels
        int id_r = has_red ? red.id : 0;
        int id_g = has_green ? green.id : 0;
        int id_b = has_blue ? blue.id : 0;
        
        return id_r + id_g * hamming_distance + id_b * hamming_distance * hamming_distance;
    }
    
    cv::Point2f getAverageCentroid() const {
        int count = 0;
        cv::Point2f sum(0, 0);
        
        if (has_red) { sum += red.centroid; count++; }
        if (has_green) { sum += green.centroid; count++; }
        if (has_blue) { sum += blue.centroid; count++; }
        
        return sum / static_cast<float>(count);
    }
    
    std::vector<cv::Point2f> getAverageCorners() const {
        // Average corners from available channels
        std::vector<cv::Point2f> avg_corners(4, cv::Point2f(0, 0));
        int count = 0;
        
        if (has_red) {
            for (size_t i = 0; i < 4; i++) avg_corners[i] += red.corners[i];
            count++;
        }
        if (has_green) {
            for (size_t i = 0; i < 4; i++) avg_corners[i] += green.corners[i];
            count++;
        }
        if (has_blue) {
            for (size_t i = 0; i < 4; i++) avg_corners[i] += blue.corners[i];
            count++;
        }
        
        for (auto& corner : avg_corners) {
            corner /= static_cast<float>(count);
        }
        
        return avg_corners;
    }
};

/**
 * @brief Base class for detection strategies
 */
class STagDetectionStrategy {
public:
    explicit STagDetectionStrategy(int hamming_distance, int error_correction = -1)
        : hamming_distance_(hamming_distance), error_correction_(error_correction) {}
    
    virtual ~STagDetectionStrategy() = default;
    
    /**
     * @brief Detect markers in the image
     * @param image Input image (BGR)
     * @return Detection result with corners, ids, and rejected candidates
     */
    virtual DetectionResult detect(const cv::Mat& image) = 0;
    
    int getHammingDistance() const { return hamming_distance_; }
    int getErrorCorrection() const { return error_correction_; }

protected:
    int hamming_distance_;
    int error_correction_;
    
    /**
     * @brief Merge two detection results
     */
    DetectionResult mergeResults(const DetectionResult& r1, const DetectionResult& r2);
    
    /**
     * @brief Merge multiple detection results
     */
    DetectionResult mergeMultipleResults(const std::vector<DetectionResult>& results);
};

/**
 * @brief HD (High Definition) detector - standard black and white markers
 */
class HDDetector : public STagDetectionStrategy {
public:
    explicit HDDetector(int hamming_distance, int error_correction = -1)
        : STagDetectionStrategy(hamming_distance, error_correction) {}
    
    DetectionResult detect(const cv::Mat& image) override;
};

/**
 * @brief HG (Hue Greyscale) detector - detects on original and inverted images
 * Uses parallel processing for 2x speedup
 */
class HGDetector : public STagDetectionStrategy {
public:
    explicit HGDetector(int hamming_distance, int error_correction = -1)
        : STagDetectionStrategy(hamming_distance, error_correction) {}
    
    DetectionResult detect(const cv::Mat& image) override;
};

/**
 * @brief HC (High Capacity) detector - RGB channel-based detection
 * Uses parallel processing on 3 channels for 3x speedup
 * Implements algorithm: HC = IDr + IDg*|HD| + IDb*|HD|²
 */
class HCDetector : public STagDetectionStrategy {
public:
    explicit HCDetector(int hamming_distance, 
                        int error_correction = -1,
                        double proximity_threshold = 10.0)
        : STagDetectionStrategy(hamming_distance, error_correction)
        , proximity_threshold_(proximity_threshold)
    {}
    
    DetectionResult detect(const cv::Mat& image) override;
    
    double getProximityThreshold() const { return proximity_threshold_; }
    void setProximityThreshold(double threshold) { proximity_threshold_ = threshold; }

private:
    double proximity_threshold_;  // Maximum distance for matching centroids (pixels)
};

/**
 * @brief HO (High Occlusion) detector - occlusion-resistant markers
 * Uses parallel processing on 3 channels with merging
 */
class HODetector : public STagDetectionStrategy {
public:
    explicit HODetector(int hamming_distance, int error_correction = -1)
        : STagDetectionStrategy(hamming_distance, error_correction) {}
    
    DetectionResult detect(const cv::Mat& image) override;
};

/**
 * @brief Factory function to create detector based on marker type
 * @param marker_type Marker type string (e.g., "HD17", "HC21")
 * @param error_correction Error correction level (-1 for default)
 * @param proximity_threshold Maximum distance for matching centroids in HC detection (pixels)
 */
std::unique_ptr<STagDetectionStrategy> createDetector(
    const std::string& marker_type,
    int error_correction = -1,
    double proximity_threshold = 10.0
);

/**
 * @brief Utility functions for detection visualization and filtering
 */
namespace utils {

/**
 * @brief Draw detections on image with annotations
 */
void drawDetections(
    cv::Mat& image,
    const DetectionResult& result,
    bool show_rejected = true,
    const std::string& marker_type = ""
);

/**
 * @brief Calculate center points of detected markers
 */
std::vector<cv::Point2f> calculateMarkerCenters(
    const std::vector<std::vector<cv::Point2f>>& corners
);

/**
 * @brief Filter duplicate detections based on spatial proximity
 */
DetectionResult filterDuplicateDetections(
    const DetectionResult& result,
    double distance_threshold = 10.0
);

} // namespace utils

} // namespace stag_ros2

#endif  // STAG_ROS2_ENHANCED__DETECTION_STRATEGIES_HPP_
