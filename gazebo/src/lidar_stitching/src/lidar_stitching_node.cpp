#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <memory>
#include <string>
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <map>
#include <mutex>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// PCL conversions
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <Eigen/Dense>

class LidarStitchingNode : public rclcpp::Node {
public:
    LidarStitchingNode() : Node("lidar_stitching_node"), tf_buffer(this->get_clock()), tf_listener(tf_buffer) {
        this->declare_parameter<std::vector<std::string>>("lidar_topics", std::vector<std::string>());
        this->declare_parameter<bool>("use_motion_compensation", false);
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("output_topic", "/lidar/points_joined");
        this->declare_parameter<double>("robot_filter_radius", 0.5);  // Радиус фильтрации точек вокруг робота (метры)
        
        std::vector<std::string> lidar_topics;
        this->get_parameter("lidar_topics", lidar_topics);
        this->get_parameter("use_motion_compensation", use_motion_compensation);
        this->get_parameter("target_frame", target_frame);
        this->get_parameter("output_topic", output_topic);
        this->get_parameter("robot_filter_radius", robot_filter_radius);
        
        // Fallback topics of the real robot, used only when lidar_topics is empty.
        // The simulation sets /lidar_front/points and /lidar_back/points in config/lidar_stitching_params.yaml.
        if (lidar_topics.empty()) {
            RCLCPP_WARN(this->get_logger(), "No lidar_topics parameter provided, using default topics");
            lidar_topics = {"/ouster_top/points_unwarped", "/ouster_left/points_unwarped", 
                           "/ouster_right/points_unwarped", "/lslidar_point_cloud"};
        }
        
        RCLCPP_INFO(this->get_logger(), "Number of LiDAR topics: %zu", lidar_topics.size());
        for (const auto& topic : lidar_topics) {
            RCLCPP_INFO(this->get_logger(), "  - %s", topic.c_str());
        }
        
        pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 1);
        
        for (const auto& topic : lidar_topics) {
            auto subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic,
                rclcpp::SensorDataQoS(),
                [this, topic](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    this->pcd_clb(topic, *msg);
                }
            );
            subs[topic] = subscription;
        }
        
        onInit();
    }

private:
    void onInit() {
        RCLCPP_INFO(this->get_logger(), "LidarStitchingNode initialized");
    }
    
    void pcd_clb(const std::string& topic, const sensor_msgs::msg::PointCloud2& msg) {
        std::lock_guard<std::mutex> lock(m);
        
        geometry_msgs::msg::TransformStamped ros_T_ol_m;
        geometry_msgs::msg::TransformStamped ros_T_bo;
        Eigen::Isometry3d T_bo;
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_nf(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(msg, *pcd_nf);
        
        // Apply passthrough filter to remove points near zero (x-axis filter)
        pcl::PointCloud<pcl::PointXYZI> pcd_filtered;
        {
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(pcd_nf);
            pass.setFilterFieldName("x");
            pass.setNegative(true);
            pass.setFilterLimits(-0.0001, 0.0001);
            pass.filter(pcd_filtered);
        }
        
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
        auto lookup_with_fallback = [this](
            const std::string& target_frame_id,
            const std::string& source_frame_id,
            const builtin_interfaces::msg::Time& stamp) {
            const auto timeout = rclcpp::Duration::from_seconds(0.05);
            try {
                return tf_buffer.lookupTransform(
                    target_frame_id,
                    source_frame_id,
                    stamp,
                    timeout
                );
            } catch (const tf2::ExtrapolationException& ex) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "TF extrapolation for %s->%s at stamp %.3f, fallback to latest transform",
                    source_frame_id.c_str(),
                    target_frame_id.c_str(),
                    rclcpp::Time(stamp).seconds()
                );
                return tf_buffer.lookupTransform(
                    target_frame_id,
                    source_frame_id,
                    rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
                    timeout
                );
            }
        };

        if (use_motion_compensation) {
            try {
                // Sensor frame -> odom (maps lidar points into odom) at the message stamp
                ros_T_ol_m = lookup_with_fallback(
                    "odom", 
                    msg.header.frame_id,
                    msg.header.stamp
                );
                
                // odom -> target_frame (maps odom points into target_frame)
                ros_T_bo = lookup_with_fallback(
                    target_frame,
                    "odom",
                    msg.header.stamp
                );
                
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
                RCLCPP_DEBUG(this->get_logger(), "Available frames: %s", tf_buffer.allFramesAsString().c_str());
                return;
            }
            
            T_oli_m[topic] = tf2::transformToEigen(ros_T_ol_m);
            T_bo = tf2::transformToEigen(ros_T_bo);
            // Transform filtered cloud to odom frame
            pcl::transformPointCloud(pcd_filtered, transformed_cloud, T_oli_m[topic].matrix());
        }
        else {
            // Lidar frame -> target_frame (maps lidar points into target_frame)
            ros_T_bo = lookup_with_fallback(target_frame, msg.header.frame_id, msg.header.stamp);
            T_bo = tf2::transformToEigen(ros_T_bo);
            transformed_cloud = pcd_filtered;
        }
        pcds[topic] = transformed_cloud;
        
        // Publish only on messages from the alphabetically first topic (subs is a std::map);
        // other topics contribute their latest stored cloud.
        if (topic == subs.begin()->first) {
            pcl::PointCloud<pcl::PointXYZI> joined;
            
            // Combine all available clouds
            for (const auto& [t, cloud] : pcds) {
                joined += cloud;
            }
            
            if (!joined.empty()) {
                // Transform joined cloud to base_link frame
                pcl::PointCloud<pcl::PointXYZI> joined_b;
                pcl::transformPointCloud(joined, joined_b, T_bo.matrix());
                    
                // Apply spatial filter to remove points near the vehicle
                // Используем радиус (круг) вместо квадрата для более точной фильтрации
                pcl::PointCloud<pcl::PointXYZI> joined_bb;
                double filter_radius_sq = robot_filter_radius * robot_filter_radius;  // Квадрат радиуса для сравнения
                for (const auto& pt : joined_b) {
                    // Вычисляем расстояние от центра робота (0, 0) до точки
                    double dist_sq = pt.x * pt.x + pt.y * pt.y;
                    // Удаляем точки в радиусе robot_filter_radius вокруг робота (в плоскости XY)
                    // и точки выше 2.0 м по Z
                    if (dist_sq > filter_radius_sq && pt.z < 2.0) {
                        joined_bb.push_back(pt);
                    }
                }
                                
                sensor_msgs::msg::PointCloud2 msg_output;
                pcl::toROSMsg(joined_bb, msg_output);
                
                msg_output.header.stamp = ros_T_bo.header.stamp;
                msg_output.header.frame_id = target_frame;
                pub->publish(msg_output);
                
                RCLCPP_DEBUG(this->get_logger(), "Published stitched cloud with %zu points", joined_bb.size());
            }
        }
    }
    
    // ROS2 members
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subs;
    
    // Data storage
    std::map<std::string, pcl::PointCloud<pcl::PointXYZI>> pcds;
    std::map<std::string, Eigen::Isometry3d> T_oli_m;
    
    // TF2
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    // Params
    bool use_motion_compensation;
    std::string target_frame;
    std::string output_topic;
    double robot_filter_radius;  // Радиус фильтрации точек вокруг робота (метры)
    
    // Mutex for thread safety
    std::mutex m;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarStitchingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}