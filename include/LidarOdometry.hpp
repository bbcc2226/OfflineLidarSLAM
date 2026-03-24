#ifndef __NDT_LO__
#define __NDT_LO__

#include "NDT_INC.hpp"
#include "Common.hpp"
#include "VoxelFilter.hpp"
#include "Config.hpp"
#include <cmath>
#include <deque>

class LidarOdodmetry{
public:
    explicit LidarOdodmetry(const double ndt_resolution):ndt_inc_(ndt_resolution){}

    std::pair<bool,Se3> AddCloud(std::shared_ptr<PointCloud>& filtered_cloud_ptr, std::shared_ptr<PointCloud>& raw_cloud_ptr, const Se3& predicted_pose, bool use_lo = false);

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
    std::deque<Se3> est_pose_buffer_;
    
    bool KeyFrameCheck(const Se3& input_pose);

    void SaveFrame(const std::shared_ptr<PointCloud>& cloud);


};




#endif