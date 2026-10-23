#pragma once

#include <vector>
#include "DataType.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class MapPublisher : public rclcpp::Node {
public:
    MapPublisher()
        : Node("voxel_map_publisher") {
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/voxel_map", rclcpp::QoS(1).transient_local());
    }

    void publish(
        const PointCloud& cloud,
        const double timestamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp.sec = static_cast<int32_t>(timestamp);
        msg.header.stamp.nanosec = static_cast<uint32_t>((timestamp - msg.header.stamp.sec) * 1e9);
        msg.header.frame_id = "map";

        msg.height = 1;
        msg.width = cloud.pt_list_.size();
        msg.is_dense = true;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(cloud.pt_list_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg, "intensity");

        for (const auto& p : cloud.pt_list_) {
            *iter_x = static_cast<float>(p[0]);
            *iter_y = static_cast<float>(p[1]);
            *iter_z = static_cast<float>(p[2]);
            *iter_intensity = static_cast<float>(p.intensity_);
            ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
        }

        map_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
};
