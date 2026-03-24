#pragma once
// or use traditional include guard
// #ifndef COMMON_HPP
// #define COMMON_HPP

#include <memory>
#include <string>
#include <vector>
#include "DataType.hpp"
#include "VoxelFilter.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

// Load KITTI binary file into PointCloud
std::shared_ptr<PointCloud> LoadKittiBin(const std::string& filename);

// Apply SE3 transform to a point cloud, returns a new cloud
std::shared_ptr<PointCloud> ApplyTransform(Sophus::SE3d& T,
                                           std::shared_ptr<PointCloud> input_point_cloud);

// Apply SE3 transform in-place
void InplaceApplyTransform(Sophus::SE3d& T,
                           std::shared_ptr<PointCloud>& input_point_cloud);

// Apply voxel downsampling filter
std::shared_ptr<PointCloud> ApplyDownSampleFilter(std::shared_ptr<PointCloud> input_cloud_ptr);


// apply range filter for point cloud
std::shared_ptr<PointCloud> ApplyRangeFilter(std::shared_ptr<PointCloud> input_cloud_ptr);


std::vector<Eigen::Vector3d> ComputeNormalsKNN(const std::vector<Eigen::Vector3d>& points, const int k = 20);

// Generate frame file path
std::string GenerateFramePath(const std::string folder_path, const int frame_count);

// Save cloud to PLY
void SaveCloud(const std::shared_ptr<PointCloud>& cloud,
               const std::string& path,
               bool filter_ground = false);

// Load PLY file
bool LoadPLY(const std::string& filename,
             std::vector<Vec3>& points);

// #endif // COMMON_HPP
