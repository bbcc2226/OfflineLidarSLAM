#pragma once

#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>
#include "DataType.hpp"

class VoxelizedMap {
public:
    explicit VoxelizedMap(double voxel_size);

    void InsertPoint(const Vec3& p);

    void InsertCloud(std::vector<Vec3>& cloud, const Se3& sensor_pose,
                     const double timestamp, const int frame_id);

    void RemoveCloud(std::vector<Vec3>& cloud, const int frame_id);

    std::pair<double, std::vector<Vec3>> GetPointCloud() const;

    void RemoveSparseVoxels(int min_neighbors = 4);

    void Clear();

private:
    struct VoxelData {
        Vec3 sum = Vec3::Zero();
        int count = 0;

        void AddPoint(const Vec3& p);
        void RemovePoint(const Vec3& p);
        bool Empty() const;
        Eigen::Vector3d Centroid() const;
    };

    using VoxelMap = std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash>;

    VoxelKey toKey(const Vec3& p) const;

    double voxel_size_;
    VoxelMap voxel_map_;
    double timestamp_;
    mutable std::mutex cloud_mtx_;
    std::unordered_map<int, Se3> key_frame_poses_;
};
