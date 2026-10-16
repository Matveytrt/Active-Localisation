#ifndef STAG_ROS2_ENHANCED__POSE_ESTIMATION_HPP_
#define STAG_ROS2_ENHANCED__POSE_ESTIMATION_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <optional>
#include <vector>

namespace stag_ros2 {

/**
 * @brief Structure to hold 6DOF pose
 */
struct Pose {
    cv::Vec3d translation;
    cv::Vec3d rotation;  // Rodrigues rotation vector
    
    Pose() : translation(0, 0, 0), rotation(0, 0, 0) {}
    Pose(const cv::Vec3d& t, const cv::Vec3d& r) : translation(t), rotation(r) {}
};

/**
 * @brief Pose estimator for STag markers
 */
class PoseEstimator {
public:
    /**
     * @brief Estimate 6DOF pose of a marker using PnP
     * @param corners Detected marker corners (4 points in image plane)
     * @param marker_size Physical size of marker in meters
     * @param camera_matrix Camera intrinsic matrix (3x3)
     * @param dist_coeffs Camera distortion coefficients
     * @return Pose if successful, nullopt otherwise
     */
    static std::optional<Pose> estimateMarkerPose(
        const std::vector<cv::Point2f>& corners,
        double marker_size,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs
    );
    
    /**
     * @brief Convert Rodrigues rotation vector to quaternion
     * @param rvec Rotation vector from solvePnP
     * @return Quaternion (x, y, z, w)
     */
    static tf2::Quaternion rotationVectorToQuaternion(const cv::Vec3d& rvec);
    
    /**
     * @brief Create ROS2 TransformStamped message from pose
     * @param pose Estimated 6DOF pose
     * @param marker_id Marker ID
     * @param parent_frame Parent frame ID (e.g., "camera_link")
     * @param child_frame_prefix Child frame prefix (e.g., "stag_marker_")
     * @param stamp Timestamp for transform
     * @return TransformStamped message
     */
    static geometry_msgs::msg::TransformStamped createTransformStamped(
        const Pose& pose,
        int marker_id,
        const std::string& parent_frame,
        const std::string& child_frame_prefix,
        const rclcpp::Time& stamp
    );

private:
    /**
     * @brief Generate 3D object points for marker corners
     * Assumes marker lies in XY plane with center at origin
     * @param marker_size Physical size of marker
     * @return Vector of 4 3D points
     */
    static std::vector<cv::Point3f> generateObjectPoints(double marker_size);
};

} // namespace stag_ros2

#endif  // STAG_ROS2_ENHANCED__POSE_ESTIMATION_HPP_
