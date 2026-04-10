#ifndef __VOXELDOWNSAMPLE__
#define __VOXELDOWNSAMPLE__

#include "DataType.hpp"
#include "ConfigManager.hpp"
#include <memory>

class VoxelFilter{
private:
    double resolution_;
    double x_range_ {ConfigManager::Get().LidarOdometry_.lidar_x_range};
    double y_range_ {ConfigManager::Get().LidarOdometry_.lidar_y_range};
    double z_range_ {ConfigManager::Get().LidarOdometry_.lidar_z_range};
    struct Accumulator {
        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        int count = 0;
    };

    // map current point to voxel key
    VoxelKey PointToKey(const Point3D& pt){
        return {
            static_cast<int>(std::floor(pt[0] / resolution_)),
            static_cast<int>(std::floor(pt[1] / resolution_)),
            static_cast<int>(std::floor(pt[2] / resolution_)),            
        };
    }

    VoxelKey PointToKey(const Eigen::Vector3d& pt){
        return {
            static_cast<int>(std::floor(pt[0] / resolution_)),
            static_cast<int>(std::floor(pt[1] / resolution_)),
            static_cast<int>(std::floor(pt[2] / resolution_)),             
        };
    }
public:
    explicit VoxelFilter(const double resolution):resolution_(resolution){}

    std::shared_ptr<PointCloud> Downsample(std::shared_ptr<PointCloud> input_ptr,bool range_filter = true){
        std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> voxel_map;
        voxel_map.reserve(input_ptr->pt_list_.size() / 3);

        for (const auto& p : input_ptr->pt_list_) {
            if(range_filter && (std::fabs(p[0]) > x_range_ || std::fabs(p[1]) > y_range_ || std::fabs(p[2]) > z_range_)){
                continue;
            }
            VoxelKey key = PointToKey(p);

            auto& acc = voxel_map[key];
            acc.sum += Eigen::Vector3d(p[0], p[1], p[2]);
            acc.count++;
        }

        auto output_ptr = std::make_shared<PointCloud>();
        output_ptr->pt_list_.reserve(voxel_map.size());
        for (const auto& [_, acc] : voxel_map) {
            Eigen::Vector3d c = acc.sum / acc.count;
            Point3D p;
            p[0] = c.x();
            p[1] = c.y();
            p[2] = c.z();
            output_ptr->pt_list_.push_back(p);
        }

        return output_ptr;
    }
};



#endif