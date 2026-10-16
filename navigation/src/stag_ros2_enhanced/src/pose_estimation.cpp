/**
 * @file pose_estimation.cpp
 * @brief Pose estimation for STag markers using OpenCV solvePnP
 */

#include "stag_ros2_enhanced/pose_estimation.hpp"
#include <opencv2/calib3d.hpp>

namespace stag_ros2 {

std::vector<cv::Point3f> PoseEstimator::generateObjectPoints(double marker_size)
{
    // Generate 3D coordinates for marker corners
    // Marker is in XY plane with center at origin
    double half_size = marker_size / 2.0;
    
    // SOLVEPNP_IPPE_SQUARE requires this exact order (OpenCV / ArUco convention):
    // Y-axis points UP in 3D space (opposite to image Y which goes down).
    // This makes markers Z-axis point toward the camera (out of marker face).
    std::vector<cv::Point3f> object_points = {
        cv::Point3f(-half_size,  half_size, 0),  // [0] top-left     in image -> (-X, +Y) in 3D
        cv::Point3f( half_size,  half_size, 0),  // [1] top-right    in image -> (+X, +Y) in 3D
        cv::Point3f( half_size, -half_size, 0),  // [2] bottom-right in image -> (+X, -Y) in 3D
        cv::Point3f(-half_size, -half_size, 0)   // [3] bottom-left  in image -> (-X, -Y) in 3D
    };
    
    return object_points;
}

std::optional<Pose> PoseEstimator::estimateMarkerPose(
    const std::vector<cv::Point2f>& corners,
    double marker_size,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs)
{
    if (corners.size() != 4) {
        return std::nullopt;
    }
    
    if (marker_size <= 0.0) {
        return std::nullopt;
    }
    
    if (camera_matrix.empty() || camera_matrix.rows != 3 || camera_matrix.cols != 3) {
        return std::nullopt;
    }
    
    // Generate 3D object points
    auto object_points = generateObjectPoints(marker_size);
    
    // Solve PnP to get pose
    cv::Vec3d rvec, tvec;
    bool success = cv::solvePnP(
        object_points,
        corners,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
        false,  // useExtrinsicGuess
        cv::SOLVEPNP_IPPE_SQUARE  // Use IPPE for planar targets
    );
    
    if (!success) {
        return std::nullopt;
    }
    
    return Pose(tvec, rvec);
}

tf2::Quaternion PoseEstimator::rotationVectorToQuaternion(const cv::Vec3d& rvec)
{
    // Convert Rodrigues rotation vector to rotation matrix
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);
    
    // Convert rotation matrix to quaternion
    // Using formula from rotation matrix to quaternion conversion
    double trace = rotation_matrix.at<double>(0, 0) +
                   rotation_matrix.at<double>(1, 1) +
                   rotation_matrix.at<double>(2, 2);
    
    tf2::Quaternion quat;
    
    if (trace > 0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        quat.setW(0.25 / s);
        quat.setX((rotation_matrix.at<double>(2, 1) - rotation_matrix.at<double>(1, 2)) * s);
        quat.setY((rotation_matrix.at<double>(0, 2) - rotation_matrix.at<double>(2, 0)) * s);
        quat.setZ((rotation_matrix.at<double>(1, 0) - rotation_matrix.at<double>(0, 1)) * s);
    }
    else {
        if (rotation_matrix.at<double>(0, 0) > rotation_matrix.at<double>(1, 1) &&
            rotation_matrix.at<double>(0, 0) > rotation_matrix.at<double>(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + rotation_matrix.at<double>(0, 0) -
                                       rotation_matrix.at<double>(1, 1) -
                                       rotation_matrix.at<double>(2, 2));
            quat.setW((rotation_matrix.at<double>(2, 1) - rotation_matrix.at<double>(1, 2)) / s);
            quat.setX(0.25 * s);
            quat.setY((rotation_matrix.at<double>(0, 1) + rotation_matrix.at<double>(1, 0)) / s);
            quat.setZ((rotation_matrix.at<double>(0, 2) + rotation_matrix.at<double>(2, 0)) / s);
        }
        else if (rotation_matrix.at<double>(1, 1) > rotation_matrix.at<double>(2, 2)) {
            double s = 2.0 * std::sqrt(1.0 + rotation_matrix.at<double>(1, 1) -
                                       rotation_matrix.at<double>(0, 0) -
                                       rotation_matrix.at<double>(2, 2));
            quat.setW((rotation_matrix.at<double>(0, 2) - rotation_matrix.at<double>(2, 0)) / s);
            quat.setX((rotation_matrix.at<double>(0, 1) + rotation_matrix.at<double>(1, 0)) / s);
            quat.setY(0.25 * s);
            quat.setZ((rotation_matrix.at<double>(1, 2) + rotation_matrix.at<double>(2, 1)) / s);
        }
        else {
            double s = 2.0 * std::sqrt(1.0 + rotation_matrix.at<double>(2, 2) -
                                       rotation_matrix.at<double>(0, 0) -
                                       rotation_matrix.at<double>(1, 1));
            quat.setW((rotation_matrix.at<double>(1, 0) - rotation_matrix.at<double>(0, 1)) / s);
            quat.setX((rotation_matrix.at<double>(0, 2) + rotation_matrix.at<double>(2, 0)) / s);
            quat.setY((rotation_matrix.at<double>(1, 2) + rotation_matrix.at<double>(2, 1)) / s);
            quat.setZ(0.25 * s);
        }
    }
    
    return quat;
}

geometry_msgs::msg::TransformStamped PoseEstimator::createTransformStamped(
    const Pose& pose,
    int marker_id,
    const std::string& parent_frame,
    const std::string& child_frame_prefix,
    const rclcpp::Time& stamp)
{
    geometry_msgs::msg::TransformStamped transform;
    
    // Set header
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame;
    transform.child_frame_id = child_frame_prefix + std::to_string(marker_id);
    
    // Set translation
    transform.transform.translation.x = pose.translation[0];
    transform.transform.translation.y = pose.translation[1];
    transform.transform.translation.z = pose.translation[2];
    
    // Convert rotation vector to quaternion
    auto quat = rotationVectorToQuaternion(pose.rotation);
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();
    
    return transform;
}

} // namespace stag_ros2
