#ifndef OFFLINE_LIDAR_SLAM_TOOLS_KEYFRAME_JSON_RECORD_HPP_
#define OFFLINE_LIDAR_SLAM_TOOLS_KEYFRAME_JSON_RECORD_HPP_

#include <string>

#include <DataType.hpp>

struct KeyFrameJsonRecord {
    int key_frame_id{-1};
    double timestamp{0.0};
    std::string saved_frame_path;
    bool valid_rtk{false};
    Vec3 rtk_pos{Vec3::Zero()};
    double rtk_align_yaw{0.0};
    Se3 lio_pose;
    Se3 optimized_pose;
};

#endif
