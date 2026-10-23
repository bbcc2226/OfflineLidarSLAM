#pragma once

#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>
#include "DataType.hpp"

class VoxelizedMap {
public:
    explicit VoxelizedMap(double voxel_size);

    void InsertPoint(const Point3D& p);

    void InsertCloud(PointCloud& cloud, const Se3& sensor_pose,
                     const double timestamp, const int frame_id);

    void RemoveCloud(PointCloud& cloud, const int frame_id);

    std::pair<double, PointCloud> GetPointCloud() const;

    void RemoveSparseVoxels(int min_neighbors = 4);

    void Clear();

private:
    struct VoxelData {
        Vec3 sum = Vec3::Zero();
        double intensity_sum = 0.0;
        int count = 0;

        void AddPoint(const Point3D& p);
        void RemovePoint(const Point3D& p);
        bool Empty() const;
        Point3D Centroid() const;
    };

    using VoxelMap = std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash>;

    VoxelKey toKey(const Point3D& p) const;

    double voxel_size_;
    VoxelMap voxel_map_;
    double timestamp_;
    mutable std::mutex cloud_mtx_;
    std::unordered_map<int, Se3> key_frame_poses_;
};
