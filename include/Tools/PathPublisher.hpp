#pragma once

#include <queue>
#include "DataType.hpp"
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
        rtk_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rtk_trajectory", 10);
    }

    void AddPathPoint(const Vec3& pos) {
        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = 1.0;

        lio_path_.header.stamp = this->now();
        lio_path_.header.frame_id = "map";

        lio_path_.poses.push_back(pose);

        path_pub_->publish(lio_path_);
    }

    void AddRTKPathPoint(const Vec3& pos, bool valid, double gps_align_yaw) {
        if (!valid) {
            rtk_pos_buffer_.push(pos);
            return;
        } else {
            rtk_pos_buffer_.push(pos);
            while (!rtk_pos_buffer_.empty()) {
                Vec3 buffered_pos = rtk_pos_buffer_.front();
                rtk_pos_buffer_.pop();
                Eigen::AngleAxisd yaw_rot(gps_align_yaw, Eigen::Vector3d::UnitZ());
                auto corrected_gps_pos = yaw_rot * buffered_pos;

                geometry_msgs::msg::PoseStamped buffered_pose;
                buffered_pose.header.stamp = this->now();
                buffered_pose.header.frame_id = "map";

                buffered_pose.pose.position.x = corrected_gps_pos.x();
                buffered_pose.pose.position.y = corrected_gps_pos.y();
                buffered_pose.pose.position.z = 0.0;

                buffered_pose.pose.orientation.w = 1.0;

                rtk_path_.header.stamp = this->now();
                rtk_path_.header.frame_id = "map";

                rtk_path_.poses.push_back(buffered_pose);
            }

            rtk_pub_->publish(rtk_path_);
        }
    }

    void PopFront() {
        if (!lio_path_.poses.empty()) {
            lio_path_.poses.erase(lio_path_.poses.begin());
        }

        if (!rtk_path_.poses.empty()) {
            rtk_path_.poses.erase(rtk_path_.poses.begin());
        }
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rtk_pub_;
    std::queue<Vec3> rtk_pos_buffer_;
    nav_msgs::msg::Path lio_path_;
    nav_msgs::msg::Path rtk_path_;
    rclcpp::TimerBase::SharedPtr timer_;
};
