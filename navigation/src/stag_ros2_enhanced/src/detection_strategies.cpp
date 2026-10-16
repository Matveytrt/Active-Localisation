/**
 * @file detection_strategies.cpp
 * @brief Implementation of STag detection strategies with parallel processing
 */

#include "stag_ros2_enhanced/detection_strategies.hpp"
#include "Stag.h"
#include <algorithm>
#include <cmath>

namespace stag_ros2 {

// ============================================================================
// STagDetectionStrategy Base Class
// ============================================================================

DetectionResult STagDetectionStrategy::mergeResults(
    const DetectionResult& r1,
    const DetectionResult& r2)
{
    DetectionResult merged;
    
    // Merge corners
    merged.corners = r1.corners;
    merged.corners.insert(
        merged.corners.end(),
        r2.corners.begin(),
        r2.corners.end()
    );
    
    // Merge IDs
    merged.ids = r1.ids;
    merged.ids.insert(
        merged.ids.end(),
        r2.ids.begin(),
        r2.ids.end()
    );
    
    // Merge rejected
    merged.rejected = r1.rejected;
    merged.rejected.insert(
        merged.rejected.end(),
        r2.rejected.begin(),
        r2.rejected.end()
    );
    
    return merged;
}

DetectionResult STagDetectionStrategy::mergeMultipleResults(
    const std::vector<DetectionResult>& results)
{
    DetectionResult merged;
    
    for (const auto& result : results) {
        merged.corners.insert(
            merged.corners.end(),
            result.corners.begin(),
            result.corners.end()
        );
        
        merged.ids.insert(
            merged.ids.end(),
            result.ids.begin(),
            result.ids.end()
        );
        
        merged.rejected.insert(
            merged.rejected.end(),
            result.rejected.begin(),
            result.rejected.end()
        );
    }
    
    return merged;
}

// ============================================================================
// HDDetector - Standard Detection
// ============================================================================

DetectionResult HDDetector::detect(const cv::Mat& image)
{
    DetectionResult result;
    
    stag::detectMarkers(
        image,
        hamming_distance_,
        result.corners,
        result.ids,
        error_correction_,
        result.rejected
    );
    
    return result;
}

// ============================================================================
// HGDetector - Parallel Detection on Original and Inverted
// ============================================================================

DetectionResult HGDetector::detect(const cv::Mat& image)
{
    DetectionResult result1, result2;
    
    // Create inverted image using bitwise NOT for better clarity
    // Alternative: cv::Mat inverse = 255 - image; (equivalent for 8-bit images)
    cv::Mat inverse;
    cv::bitwise_not(image, inverse);
    
    // Create local copies to ensure thread safety
    // cv::Mat uses reference counting, but we clone to avoid any race conditions
    cv::Mat image_copy = image.clone();
    cv::Mat inverse_copy = inverse.clone();
    
    // Parallel detection on original and inverted images
    std::thread t1([&, image_copy]() {
        try {
            stag::detectMarkers(
                image_copy,
                hamming_distance_,
                result1.corners,
                result1.ids,
                error_correction_,
                result1.rejected
            );
        } catch (const cv::Exception& e) {
            // OpenCV error in thread - clear results
            result1.corners.clear();
            result1.ids.clear();
            result1.rejected.clear();
        } catch (const std::exception& e) {
            // Other error - clear results
            result1.corners.clear();
            result1.ids.clear();
            result1.rejected.clear();
        }
    });
    
    std::thread t2([&, inverse_copy]() {
        try {
            stag::detectMarkers(
                inverse_copy,
                hamming_distance_,
                result2.corners,
                result2.ids,
                error_correction_,
                result2.rejected
            );
        } catch (const cv::Exception& e) {
            // OpenCV error in thread - clear results
            result2.corners.clear();
            result2.ids.clear();
            result2.rejected.clear();
        } catch (const std::exception& e) {
            // Other error - clear results
            result2.corners.clear();
            result2.ids.clear();
            result2.rejected.clear();
        }
    });
    
    // Wait for both threads to complete
    t1.join();
    t2.join();
    
    // Merge results
    return mergeResults(result1, result2);
}

// ============================================================================
// Helper functions for HC detection
// ============================================================================

namespace {

/**
 * @brief Compute centroid of marker corners
 */
cv::Point2f computeCentroid(const std::vector<cv::Point2f>& corners) {
    cv::Point2f centroid(0, 0);
    for (const auto& corner : corners) {
        centroid += corner;
    }
    return centroid / static_cast<float>(corners.size());
}

/**
 * @brief Convert DetectionResult to ChannelDetection list
 */
std::vector<stag_ros2::ChannelDetection> convertToChannelDetections(
    const stag_ros2::DetectionResult& result,
    int channel_index)
{
    std::vector<stag_ros2::ChannelDetection> detections;
    
    for (size_t i = 0; i < result.corners.size(); i++) {
        stag_ros2::ChannelDetection det;
        det.corners = result.corners[i];
        det.centroid = computeCentroid(det.corners);
        det.id = (i < result.ids.size()) ? result.ids[i] : -1;
        det.channel_index = channel_index;
        detections.push_back(det);
    }
    
    return detections;
}

/**
 * @brief Match detections across RGB channels by centroid proximity
 * Algorithm step 5: Match bboxes across R,G,B by centroid proximity p
 */
std::vector<stag_ros2::TripleMatch> matchAcrossChannels(
    const std::vector<stag_ros2::ChannelDetection>& red_detections,
    const std::vector<stag_ros2::ChannelDetection>& green_detections,
    const std::vector<stag_ros2::ChannelDetection>& blue_detections,
    double proximity_threshold)
{
    std::vector<stag_ros2::TripleMatch> matches;
    std::vector<bool> green_matched(green_detections.size(), false);
    std::vector<bool> blue_matched(blue_detections.size(), false);
    
    // Start with red channel detections as base
    for (const auto& red_det : red_detections) {
        stag_ros2::TripleMatch match;
        match.red = red_det;
        match.has_red = true;
        
        // Find closest green detection
        double min_green_dist = std::numeric_limits<double>::max();
        int green_idx = -1;
        for (size_t i = 0; i < green_detections.size(); i++) {
            if (green_matched[i]) continue;
            double dist = cv::norm(red_det.centroid - green_detections[i].centroid);
            if (dist < min_green_dist && dist < proximity_threshold) {
                min_green_dist = dist;
                green_idx = static_cast<int>(i);
            }
        }
        
        if (green_idx >= 0) {
            match.green = green_detections[green_idx];
            match.has_green = true;
            green_matched[green_idx] = true;
        }
        
        // Find closest blue detection
        double min_blue_dist = std::numeric_limits<double>::max();
        int blue_idx = -1;
        for (size_t i = 0; i < blue_detections.size(); i++) {
            if (blue_matched[i]) continue;
            double dist = cv::norm(red_det.centroid - blue_detections[i].centroid);
            if (dist < min_blue_dist && dist < proximity_threshold) {
                min_blue_dist = dist;
                blue_idx = static_cast<int>(i);
            }
        }
        
        if (blue_idx >= 0) {
            match.blue = blue_detections[blue_idx];
            match.has_blue = true;
            blue_matched[blue_idx] = true;
        }
        
        matches.push_back(match);
    }
    
    return matches;
}

} // anonymous namespace

// ============================================================================
// HCDetector - Parallel Detection on RGB Channels
// ============================================================================
// 
// HC (High Capacity) Marker Detection Algorithm:
// Based on scientific paper algorithm for multi-channel marker detection
// 
// Steps:
// 1. Take a 3-channel BGR image
// 2. cv::split into channels[0]=B, channels[1]=G, channels[2]=R
// 3. Run STag detector on each single-channel image in parallel (no [Ic, Ic, Ic] replication)
// 4. For each detected bbox compute centroid (x̄,ȳ)
// 5. Match bboxes across R,G,B by centroid proximity p
// 6. For each matched triple obtain IDs (IDr, IDg, IDb)
// 7. Compute HC = IDr + IDg*|HD| + IDb*|HD|²
// 
// This allows encoding much higher marker IDs by using all 3 color channels
// ============================================================================

DetectionResult HCDetector::detect(const cv::Mat& image)
{
    // Step 1: Validate input - require 3-channel BGR image
    if (image.channels() != 3) {
        throw std::runtime_error("HCDetector requires a 3-channel BGR image");
    }
    
    // Step 2: Split into BGR channels (OpenCV uses BGR, not RGB)
    // cv::split() divides multi-channel array into several single-channel arrays
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    
    // Validate split result
    if (channels.size() != 3) {
        throw std::runtime_error("HCDetector: cv::split did not produce 3 channels");
    }
    
    // Create local copies of channels for thread safety
    // Each thread gets its own copy to avoid race conditions
    cv::Mat blue_channel = channels[0].clone();   // B channel
    cv::Mat green_channel = channels[1].clone();  // G channel
    cv::Mat red_channel = channels[2].clone();    // R channel
    
    // Results for each channel (pre-allocated, each thread writes to its own index)
    std::vector<DetectionResult> results(3);
    
    // Step 3: Run STag detector on each channel
    // Each split channel is a single-channel image and is passed to STag directly
    std::vector<std::thread> threads;
    
    // Thread for Blue channel
    threads.emplace_back([&, blue_channel]() {
        try {
            stag::detectMarkers(
                blue_channel,
                hamming_distance_,
                results[0].corners,
                results[0].ids,
                error_correction_,
                results[0].rejected
            );
        } catch (const cv::Exception& e) {
            // OpenCV error in thread - clear results for this channel
            results[0].corners.clear();
            results[0].ids.clear();
            results[0].rejected.clear();
        } catch (const std::exception& e) {
            // Other error - clear results
            results[0].corners.clear();
            results[0].ids.clear();
            results[0].rejected.clear();
        }
    });
    
    // Thread for Green channel
    threads.emplace_back([&, green_channel]() {
        try {
            stag::detectMarkers(
                green_channel,
                hamming_distance_,
                results[1].corners,
                results[1].ids,
                error_correction_,
                results[1].rejected
            );
        } catch (const cv::Exception& e) {
            results[1].corners.clear();
            results[1].ids.clear();
            results[1].rejected.clear();
        } catch (const std::exception& e) {
            results[1].corners.clear();
            results[1].ids.clear();
            results[1].rejected.clear();
        }
    });
    
    // Thread for Red channel
    threads.emplace_back([&, red_channel]() {
        try {
            stag::detectMarkers(
                red_channel,
                hamming_distance_,
                results[2].corners,
                results[2].ids,
                error_correction_,
                results[2].rejected
            );
        } catch (const cv::Exception& e) {
            results[2].corners.clear();
            results[2].ids.clear();
            results[2].rejected.clear();
        } catch (const std::exception& e) {
            results[2].corners.clear();
            results[2].ids.clear();
            results[2].rejected.clear();
        }
    });
    
    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Step 4: For each detected bbox compute centroid (x̄,ȳ)
    // Convert results to ChannelDetections (centroids computed inside)
    auto blue_detections = convertToChannelDetections(results[0], 0);   // B channel
    auto green_detections = convertToChannelDetections(results[1], 1);  // G channel
    auto red_detections = convertToChannelDetections(results[2], 2);    // R channel
    
    // Step 5: Match bboxes across R,G,B by centroid proximity p
    auto matches = matchAcrossChannels(
        red_detections, 
        green_detections, 
        blue_detections,
        proximity_threshold_
    );
    
    // Step 6 & 7: For each matched triple obtain IDs and compute HC ID
    DetectionResult final_result;
    
    for (const auto& match : matches) {
        // Process matches detected in at least one channel
        // For optimal HC markers, all 3 channels should be detected
        // But we allow partial matches for robustness
        if (match.hasAny()) {
            // Step 7: Compute HC = IDr + IDg*|HD| + IDb*|HD|²
            // Missing channels contribute 0 to the ID
            int hc_id = match.computeHCId(hamming_distance_);
            
            // Use averaged corners from available channels
            final_result.corners.push_back(match.getAverageCorners());
            final_result.ids.push_back(hc_id);
        }
    }
    
    // Fallback: If no matches found, use red channel only (most common for HC markers)
    if (final_result.empty() && !red_detections.empty()) {
        for (const auto& det : red_detections) {
            final_result.corners.push_back(det.corners);
            final_result.ids.push_back(det.id);
        }
    }
    
    return final_result;
}

// ============================================================================
// HODetector - Parallel Detection with Full Merging
// ============================================================================

DetectionResult HODetector::detect(const cv::Mat& image)
{
    if (image.channels() != 3) {
        throw std::runtime_error("HODetector requires a 3-channel BGR image");
    }
    
    // Split into BGR channels using cv::split()
    std::vector<cv::Mat> channels;
    cv::split(image, channels);
    
    // Validate split result
    if (channels.size() != 3) {
        throw std::runtime_error("HODetector: cv::split did not produce 3 channels");
    }
    
    // Create thread-safe copies of channels
    cv::Mat blue_channel = channels[0].clone();
    cv::Mat green_channel = channels[1].clone();
    cv::Mat red_channel = channels[2].clone();
    
    // Results for each channel (pre-allocated)
    std::vector<DetectionResult> results(3);
    
    // Parallel detection on each channel with explicit channel copies
    std::vector<std::thread> threads;
    
    // Thread for Blue channel
    threads.emplace_back([&, blue_channel]() {
        try {
            stag::detectMarkers(
                blue_channel,
                hamming_distance_,
                results[0].corners,
                results[0].ids,
                error_correction_,
                results[0].rejected
            );
        } catch (const cv::Exception& e) {
            results[0].corners.clear();
            results[0].ids.clear();
            results[0].rejected.clear();
        } catch (const std::exception& e) {
            results[0].corners.clear();
            results[0].ids.clear();
            results[0].rejected.clear();
        }
    });
    
    // Thread for Green channel
    threads.emplace_back([&, green_channel]() {
        try {
            stag::detectMarkers(
                green_channel,
                hamming_distance_,
                results[1].corners,
                results[1].ids,
                error_correction_,
                results[1].rejected
            );
        } catch (const cv::Exception& e) {
            results[1].corners.clear();
            results[1].ids.clear();
            results[1].rejected.clear();
        } catch (const std::exception& e) {
            results[1].corners.clear();
            results[1].ids.clear();
            results[1].rejected.clear();
        }
    });
    
    // Thread for Red channel
    threads.emplace_back([&, red_channel]() {
        try {
            stag::detectMarkers(
                red_channel,
                hamming_distance_,
                results[2].corners,
                results[2].ids,
                error_correction_,
                results[2].rejected
            );
        } catch (const cv::Exception& e) {
            results[2].corners.clear();
            results[2].ids.clear();
            results[2].rejected.clear();
        } catch (const std::exception& e) {
            results[2].corners.clear();
            results[2].ids.clear();
            results[2].rejected.clear();
        }
    });
    
    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Merge all channel results without matching (simple concatenation)
    return mergeMultipleResults(results);
}

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<STagDetectionStrategy> createDetector(
    const std::string& marker_type,
    int error_correction,
    double proximity_threshold)
{
    // Parse marker type to get prefix and hamming distance
    std::string prefix;
    int hamming_distance = 0;
    
    if (marker_type.substr(0, 2) == "HD") {
        prefix = "HD";
        hamming_distance = std::stoi(marker_type.substr(2));
        return std::make_unique<HDDetector>(hamming_distance, error_correction);
    }
    else if (marker_type.substr(0, 2) == "HG") {
        prefix = "HG";
        hamming_distance = std::stoi(marker_type.substr(2));
        return std::make_unique<HGDetector>(hamming_distance, error_correction);
    }
    else if (marker_type.substr(0, 2) == "HC") {
        prefix = "HC";
        hamming_distance = std::stoi(marker_type.substr(2));
        return std::make_unique<HCDetector>(hamming_distance, error_correction, proximity_threshold);
    }
    else if (marker_type.substr(0, 2) == "HO") {
        prefix = "HO";
        hamming_distance = std::stoi(marker_type.substr(2));
        return std::make_unique<HODetector>(hamming_distance, error_correction);
    }
    
    throw std::invalid_argument("Unsupported marker type: " + marker_type);
}

// ============================================================================
// Utility Functions
// ============================================================================

namespace utils {

void drawDetections(
    cv::Mat& image,
    const DetectionResult& result,
    bool show_rejected,
    const std::string& marker_type)
{
    // Validate image
    if (image.empty()) {
        return;
    }
    
    // Draw detected markers in green
    if (!result.corners.empty() && !result.ids.empty()) {
        // Validate that all corner vectors are non-empty
        bool valid = true;
        for (const auto& corners : result.corners) {
            if (corners.empty() || corners.size() != 4) {
                valid = false;
                break;
            }
        }
        
        if (valid) {
            stag::drawDetectedMarkers(
                image,
                result.corners,
                result.ids,
                cv::Scalar(0, 255, 0)  // Green
            );
        }
        
        // Add marker ID text
        for (size_t i = 0; i < result.corners.size() && i < result.ids.size(); ++i) {
            // Calculate center
            cv::Point2f center(0, 0);
            for (const auto& point : result.corners[i]) {
                center += point;
            }
            center.x /= result.corners[i].size();
            center.y /= result.corners[i].size();
            
            // Draw ID text
            std::string text = "ID:" + std::to_string(result.ids[i]);
            int font = cv::FONT_HERSHEY_SIMPLEX;
            double font_scale = 0.6;
            int thickness = 2;
            
            // Get text size for background
            int baseline = 0;
            cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);
            
            // Draw black background
            cv::rectangle(
                image,
                cv::Point(center.x - 5, center.y - text_size.height - 5),
                cv::Point(center.x + text_size.width + 5, center.y + 5),
                cv::Scalar(0, 0, 0),
                cv::FILLED
            );
            
            // Draw white text
            cv::putText(
                image,
                text,
                cv::Point(center.x, center.y),
                font,
                font_scale,
                cv::Scalar(255, 255, 255),
                thickness
            );
        }
    }
    
    // Draw rejected markers in red
    if (show_rejected && !result.rejected.empty()) {
        // Validate that all corner vectors are non-empty
        bool valid = true;
        for (const auto& corners : result.rejected) {
            if (corners.empty() || corners.size() != 4) {
                valid = false;
                break;
            }
        }
        
        if (valid) {
            stag::drawDetectedMarkers(
                image,
                result.rejected,
                std::vector<int>(),  // No IDs for rejected
                cv::Scalar(0, 0, 255)  // Red
            );
        }
    }
}

std::vector<cv::Point2f> calculateMarkerCenters(
    const std::vector<std::vector<cv::Point2f>>& corners)
{
    std::vector<cv::Point2f> centers;
    centers.reserve(corners.size());
    
    for (const auto& corner_set : corners) {
        cv::Point2f center(0, 0);
        for (const auto& point : corner_set) {
            center += point;
        }
        center.x /= corner_set.size();
        center.y /= corner_set.size();
        centers.push_back(center);
    }
    
    return centers;
}

DetectionResult filterDuplicateDetections(
    const DetectionResult& result,
    double distance_threshold)
{
    if (result.empty()) {
        return result;
    }
    
    // Calculate centers
    auto centers = calculateMarkerCenters(result.corners);
    
    // Track which markers to keep
    std::vector<bool> keep(result.corners.size(), true);
    
    // Compare all pairs
    for (size_t i = 0; i < centers.size(); ++i) {
        if (!keep[i]) continue;
        
        for (size_t j = i + 1; j < centers.size(); ++j) {
            if (!keep[j]) continue;
            
            // Calculate distance
            double dx = centers[i].x - centers[j].x;
            double dy = centers[i].y - centers[j].y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            // If too close, mark as duplicate
            if (dist < distance_threshold) {
                keep[j] = false;
            }
        }
    }
    
    // Filter results
    DetectionResult filtered;
    for (size_t i = 0; i < result.corners.size(); ++i) {
        if (keep[i]) {
            filtered.corners.push_back(result.corners[i]);
            if (i < result.ids.size()) {
                filtered.ids.push_back(result.ids[i]);
            }
        }
    }
    
    // Keep all rejected (no filtering needed)
    filtered.rejected = result.rejected;
    
    return filtered;
}

} // namespace utils

} // namespace stag_ros2
