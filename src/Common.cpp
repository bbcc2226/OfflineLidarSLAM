#include "Common.hpp"
#include "Config.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <limits>

// -------- Load KITTI binary --------
std::shared_ptr<PointCloud> LoadKittiBin(const std::string& filename) {
    std::ifstream input(filename, std::ios::binary);
    if (!input.is_open()) {
        std::cerr << "Failed to open: " << filename << std::endl;
        return nullptr;
    }

    std::shared_ptr<PointCloud> cloud_ptr = std::make_shared<PointCloud>();
    input.seekg(0, std::ios::end);
    size_t num_bytes = input.tellg();
    input.seekg(0, std::ios::beg);
    size_t num_points = num_bytes / (4 * sizeof(float));

    cloud_ptr->pt_list_.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        float data[4];
        input.read(reinterpret_cast<char*>(data), 4 * sizeof(float));
        Point3D pt;
        pt[0] = data[0];
        pt[1] = data[1];
        pt[2] = data[2];
        cloud_ptr->pt_list_.push_back(pt);
    }

    input.close();
    return cloud_ptr;
}

// -------- Apply transform --------
std::shared_ptr<PointCloud> ApplyTransform(Sophus::SE3d& T,
                                           std::shared_ptr<PointCloud> input_point_cloud) {
    const size_t N = input_point_cloud->pt_list_.size();
    std::shared_ptr<PointCloud> trans_cloud = std::make_shared<PointCloud>();
    trans_cloud->pt_list_.reserve(N);

    for(size_t i = 0; i < N; i++){
        const auto& curr_pt = input_point_cloud->pt_list_[i];
        Eigen::Vector3d t_pt = T * Eigen::Vector3d(curr_pt[0],curr_pt[1],curr_pt[2]);
        trans_cloud->pt_list_.push_back({t_pt[0],t_pt[1],t_pt[2]});
    }
    return trans_cloud;
}

void InplaceApplyTransform(Sophus::SE3d& T,
                           std::shared_ptr<PointCloud>& input_point_cloud){
    const size_t N = input_point_cloud->pt_list_.size();
    for(size_t i = 0; i < N; i++){
        const auto& curr_pt = input_point_cloud->pt_list_[i];
        Eigen::Vector3d t_pt = T * Eigen::Vector3d(curr_pt[0],curr_pt[1],curr_pt[2]);
        input_point_cloud->pt_list_[i] = {t_pt[0],t_pt[1],t_pt[2]};
    }
}

// -------- Apply downsample filter --------
std::shared_ptr<PointCloud> ApplyDownSampleFilter(std::shared_ptr<PointCloud> input_cloud_ptr){
    VoxelFilter filter(Config::LidarOdometry::voxel_resolution);
    return filter.Downsample(input_cloud_ptr);
}

std::shared_ptr<PointCloud> ApplyRangeFilter(std::shared_ptr<PointCloud> input_cloud_ptr){
    const double x_range = 50.0;
    const double y_range = 30.0;
    const double z_range = 5.0;
    auto output_ptr = std::make_shared<PointCloud>();
    output_ptr->pt_list_.reserve(input_cloud_ptr->pt_list_.size() / 3);
    for (const auto& p : input_cloud_ptr->pt_list_) {
        if(std::fabs(p[0]) > x_range || std::fabs(p[1]) > y_range || std::fabs(p[2]) > z_range){
            continue;
        }
        output_ptr->pt_list_.push_back(p);
    }
    return output_ptr;
}


// -------- Generate frame path --------
std::string GenerateFramePath(const std::string folder_path, const int frame_count){
    std::ostringstream oss;
    oss << folder_path << "/cloud_" << std::setw(6) << std::setfill('0') << frame_count << ".ply"; 
    return oss.str();
}

// -------- Save cloud --------
void SaveCloud(const std::shared_ptr<PointCloud>& cloud,
               const std::string& path,
               bool filter_ground) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if(!filter_ground){
        pcl_cloud->reserve(cloud->pt_list_.size());
        for (const auto& pt : cloud->pt_list_) {
            pcl::PointXYZ p;
            p.x = pt[0];
            p.y = pt[1];
            p.z = pt[2];
            pcl_cloud->push_back(p);
        }
    } else {
        for (const auto& pt : cloud->pt_list_) {
            if(pt[2] > -1.0){
                pcl::PointXYZ p;
                p.x = pt[0];
                p.y = pt[1];
                p.z = pt[2];
                pcl_cloud->push_back(p);
            }
        }
    }

    pcl::io::savePLYFile(path, *pcl_cloud);
}

// -------- Load PLY file --------
bool LoadPLY(const std::string& filename,
             std::vector<Vec3>& points){
    std::ifstream fin(filename);
    if (!fin.is_open()) {
        std::cerr << "Cannot open PLY file: " << filename << std::endl;
        return false;
    }

    std::string line;
    size_t vertex_count = 0;

    while (std::getline(fin, line)) {
        if (line.find("element vertex") != std::string::npos) {
            vertex_count = std::stoul(line.substr(15));
        }
        if (line == "end_header") break;
    }

    if (vertex_count == 0) {
        std::cerr << "Invalid PLY: vertex count = 0\n";
        return false;
    }

    points.clear();
    points.reserve(vertex_count);

    for (size_t i = 0; i < vertex_count; ++i) {
        double x, y, z;
        fin >> x >> y >> z;
        points.emplace_back(x, y, z);
        fin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    return true;
}
