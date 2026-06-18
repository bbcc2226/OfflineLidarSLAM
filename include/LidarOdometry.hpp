#ifndef OFFLINE_LIDAR_SLAM_LIDAR_ODOMETRY_HPP
#define OFFLINE_LIDAR_SLAM_LIDAR_ODOMETRY_HPP

#include "NDT_INC.hpp"
#include "ScanMatching.hpp"
#include "Common.hpp"
#include <cmath>
#include <deque>

class LidarOdodmetry{
public:
    LidarOdodmetry(const double ndt_resolution_x,
                   const double ndt_resolution_y,
                   const double ndt_resolution_z)
        : ndt_inc_(ndt_resolution_x, ndt_resolution_y, ndt_resolution_z) {}

    std::pair<int,Se3> AddCloud(std::shared_ptr<PointCloud>& filtered_cloud_ptr, std::shared_ptr<PointCloud>& raw_cloud_ptr, const Se3& predicted_pose, bool use_lo = false);

    bool ValidPose() const {
        return !first_frame_;
    }

private:
    int frame_cnt_ {0};
    int total_cnt_{0};
    int key_frame_cnt_{0};
    bool first_frame_{true};
    Se3 last_kf_pose_{Se3()};
    NDT_INC ndt_inc_;
    CoarseToFineRegistration scan_matcher_{
        ConfigManager::Get().LidarOdometry_.scan_matching_gicp_max_correspondence_dist,
        ConfigManager::Get().LidarOdometry_.scan_matching_gicp_max_iter,
        ConfigManager::Get().LidarOdometry_.scan_matching_gicp_transform_epsilon,
        ConfigManager::Get().LidarOdometry_.scan_matching_gicp_fitness_epsilon};
    std::deque<Se3> est_pose_buffer_;
    
    bool PoseJumpAccepted(const Se3& guess, const Se3& estimated_pose) const;

    bool KeyFrameCheck(const Se3& input_pose);

    void SaveFrame(const std::shared_ptr<PointCloud>& cloud);

};




#endif