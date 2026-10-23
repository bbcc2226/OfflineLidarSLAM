#include "VoxelizedMap.hpp"
#include "ConfigManager.hpp"
#include <cmath>
#include <iostream>

// --- VoxelData ---

void VoxelizedMap::VoxelData::AddPoint(const Point3D& p) {
    sum += Vec3(p[0], p[1], p[2]);
    intensity_sum += p.intensity_;
    ++count;
}

void VoxelizedMap::VoxelData::RemovePoint(const Point3D& p) {
    sum -= Vec3(p[0], p[1], p[2]);
    intensity_sum -= p.intensity_;
    --count;
}

bool VoxelizedMap::VoxelData::Empty() const {
    return count == 0;
}

Point3D VoxelizedMap::VoxelData::Centroid() const {
    const Vec3 centroid = sum / static_cast<double>(count);
    return Point3D{{centroid.x(), centroid.y(), centroid.z()},
                   intensity_sum / static_cast<double>(count)};
}

// --- VoxelizedMap ---

VoxelizedMap::VoxelizedMap(double voxel_size)
    : voxel_size_(voxel_size), timestamp_(0.0) {}

VoxelKey VoxelizedMap::toKey(const Point3D& p) const {
    return VoxelKey{
        static_cast<int>(std::floor(p[0] / voxel_size_)),
        static_cast<int>(std::floor(p[1] / voxel_size_)),
        static_cast<int>(std::floor(p[2] / voxel_size_))
    };
}

void VoxelizedMap::InsertPoint(const Point3D& p) {
    VoxelKey key = toKey(p);
    voxel_map_[key].AddPoint(p);
}

void VoxelizedMap::InsertCloud(PointCloud& cloud, const Se3& sensor_pose,
                                const double timestamp, const int frame_id) {
    for (auto& point : cloud.pt_list_) {
        const Vec3 transformed =
            sensor_pose * Vec3(point[0], point[1], point[2]);
        point[0] = transformed.x();
        point[1] = transformed.y();
        point[2] = transformed.z();
    }
    std::lock_guard<std::mutex> lock(cloud_mtx_);
    for (const auto& p : cloud.pt_list_) {
        InsertPoint(p);
    }
    timestamp_ = timestamp;
    key_frame_poses_[frame_id] = sensor_pose;
}

void VoxelizedMap::RemoveCloud(PointCloud& cloud, const int frame_id) {
    const auto it = key_frame_poses_.find(frame_id);
    if (it == key_frame_poses_.end()) {
        return;
    }
    const Se3& sensor_pose = it->second;
    for (auto& point : cloud.pt_list_) {
        const Vec3 transformed =
            sensor_pose * Vec3(point[0], point[1], point[2]);
        point[0] = transformed.x();
        point[1] = transformed.y();
        point[2] = transformed.z();
    }
    std::lock_guard<std::mutex> lock(cloud_mtx_);
    for (const auto& p : cloud.pt_list_) {
        VoxelKey key = toKey(p);
        auto it = voxel_map_.find(key);
        if (it == voxel_map_.end()) {
            continue;
        }

        it->second.RemovePoint(p);

        if (it->second.Empty()) {
            voxel_map_.erase(it);
        }
    }
}

std::pair<double, PointCloud> VoxelizedMap::GetPointCloud() const {
    std::lock_guard<std::mutex> lock(cloud_mtx_);
    PointCloud cloud;
    cloud.has_intensity_ = true;
    cloud.pt_list_.reserve(voxel_map_.size());

    for (const auto& kv : voxel_map_) {
        if (kv.second.count < ConfigManager::Get().General_.num_pts_threshold_for_viz) continue;
        cloud.pt_list_.push_back(kv.second.Centroid());
    }

    return {timestamp_, cloud};
}

void VoxelizedMap::Clear() {
    std::lock_guard<std::mutex> lock(cloud_mtx_);
    voxel_map_.clear();
    key_frame_poses_.clear();
    timestamp_ = 0.0;
}

void VoxelizedMap::RemoveSparseVoxels(int min_neighbors) {
    std::lock_guard<std::mutex> lock(cloud_mtx_);
    std::vector<VoxelKey> voxels_to_remove;

    for (const auto& kv : voxel_map_) {
        const VoxelKey& key = kv.first;

        // skip voxels that don't meet visualization threshold anyway
        if (kv.second.count < ConfigManager::Get().General_.num_pts_threshold_for_viz) continue;

        int neighbor_count = 0;

        // check 26 neighbors
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;

                    VoxelKey neighbor_key{key.x_ + dx, key.y_ + dy, key.z_ + dz};
                    if (voxel_map_.find(neighbor_key) != voxel_map_.end()) {
                        ++neighbor_count;
                    }
                }
            }
        }

        if (neighbor_count < min_neighbors) {
            voxels_to_remove.push_back(key);
        }
    }

    // remove sparse voxels
    for (const auto& key : voxels_to_remove) {
        voxel_map_.erase(key);
    }

    if (!voxels_to_remove.empty()) {
        std::cout << "[VoxelizedMap] Removed " << voxels_to_remove.size()
                  << " sparse voxels.\n";
    }
}
