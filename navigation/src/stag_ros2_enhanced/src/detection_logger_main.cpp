/**
 * @file detection_logger_main.cpp
 * @brief Entry point for detection logger node
 */

#include "stag_ros2_enhanced/detection_logger_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<stag_ros2::DetectionLoggerNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("detection_logger"), "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
