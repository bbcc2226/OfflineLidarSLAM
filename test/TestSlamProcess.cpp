#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "SlamProcess.hpp"
#include "ConfigManager.hpp"

TEST(SlamProcess, Test){
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    ConfigManager::Load("/root/ros2_ws/src/offline_lidar_slam/include/Config/Config.yaml");
    SlamProcess slam;
    while (rclcpp::ok() && !slam.IsFinished()) {
        // This keeps the process alive but responsive to Ctrl+C
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();

}