#ifndef __NDT_INC__
#define __NDT_INC__

#include <execution>
#include <list>
#include <vector>
#include <unordered_map>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <unordered_set>
#include <iostream>
#include "DataType.hpp"

struct VoxelGaussian_INC {
    Vec3 mean_{Vec3::Zero()};
    Mat3 cov_{Mat3::Zero()};
    Mat3 inv_cov_{Mat3::Zero()};
    std::vector<Vec3> pts_;
    bool ndt_estimated_{false};

    int num_pts_ {0};

    void AddPoint(const Vec3& p) {
        pts_.push_back(p);
        if(!ndt_estimated_){
            num_pts_ += 1;
        }
    }
};

// incremental NDT
class NDT_INC{
public:
    explicit NDT_INC(const double resolution) : resolution_(resolution){}
    // input point cloud already in world frame
    void AddCloud(std::shared_ptr<PointCloud> input_scan_ptr);

    bool IsIntialized() const {
        return initial_;
    }

    Se3 Align(std::shared_ptr<PointCloud> input_scan_ptr,const Se3& init_pose, const int max_iter = 10);

    double ComputeFitnessScore(std::shared_ptr<PointCloud> input_scan_ptr,const Se3& pose);

private:
    const double resolution_;
    bool initial_ {false};
    using KeyAndVoxel = std::pair<VoxelKey, VoxelGaussian_INC>;
    std::list<KeyAndVoxel> data_buffer_;
    std::unordered_map<VoxelKey,std::list<KeyAndVoxel>::iterator,VoxelKeyHash> grids_;
    std::shared_ptr<PointCloud> source_scan_;

    void UpdateVoxel(VoxelGaussian_INC& voxel_data);

    void ComputeMeanAndVariance(VoxelGaussian_INC& voxel_data, Vec3& mean, Mat3& cov);

    void UpdateMeanAndVariance(VoxelGaussian_INC& voxel_data);

    VoxelKey PointToKey(const Eigen::Vector3d& pt);

};

#endif