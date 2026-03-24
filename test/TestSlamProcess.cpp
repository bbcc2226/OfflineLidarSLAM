#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "SlamProcess.hpp"

TEST(SlamProcess, Test){
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    SlamProcess slam;
    while (rclcpp::ok() && !slam.IsFinished()) {
        // This keeps the process alive but responsive to Ctrl+C
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();

}