#ifndef __DATATYPE__
#define __DATATYPE__

#include <vector>
#include <array>
#include <Eigen/Dense>
#include <memory>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix<double,3,3>;
using Mat15 = Eigen::Matrix<double,15,15>;
using Vec15 = Eigen::Matrix<double,15,1>;
using So3 = Sophus::SO3d;
using Se3 = Sophus::SE3d;

// imu sensor data [input]
struct IMUdata{
    double timestamp_{0.0};
    Vec3 acc_{Vec3::Zero()};
    Vec3 gyro_{Vec3::Zero()};
};

// gps sensor data [input]
struct GPSdata{
  double timestamp_{0.0};
  // [lat lon alt]
  Vec3 pos_{Vec3::Zero()};
  // yaw pitch row[ usually only yaw is available]
  Vec3 heading_{Vec3::Zero()};
};

// lidar sensor data [input]
struct Point3D {
    std::array<double,3> point_;

    Point3D() {
        point_.fill(0);
    }

    Point3D(std::initializer_list<double> input_vals){

        std::copy(input_vals.begin(),input_vals.end(),point_.begin());
    }

    double operator[](size_t i) const { return point_[i]; }
    double& operator[](size_t i) { return point_[i]; }

    double Dist(const Point3D& other_pt) {
        double sum = 0.0;
        for (std::size_t i = 0; i < 3; i++) {
            sum += (point_[i] - other_pt.point_[i]) * (point_[i] - other_pt.point_[i]);
        }
        return sum;
    }
};

struct PointCloud {
    double timestamp_{0.0};
    std::vector<Point3D> pt_list_;

    PointCloud() = default;
    
    explicit PointCloud(std::vector<Point3D>& input_pt_list):pt_list_(input_pt_list){}
    // Required by nanoflann
    inline size_t kdtree_get_point_count() const { return pt_list_.size(); }

    // Required by nanoflann
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pt_list_[idx][dim];
    }

    Point3D operator[](const size_t idx) const {
        return pt_list_[idx];
    }

    Point3D& operator[](const size_t idx) {
        return pt_list_[idx];
    }

    // Optional: distance metric (default is L2)
    template <class BBox>
    bool kdtree_get_bbox(BBox&) const { return false; }
};

// Datatype related to voxel downgrading and NDT
struct VoxelKey{
    int x_{0};
    int y_{0};
    int z_{0};
    bool operator==(const VoxelKey& other) const {
        return (x_ == other.x_) && (y_ == other.y_) && (z_ == other.z_);
    }
};


struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
         return ((size_t)k.x_ * 73856093) ^ ((size_t)k.y_ * 19349663) ^ ((size_t)k.z_ * 83492791);
    }
};

struct PointcloudComb{
    int frame_id_ {0};
    double timestamp_ {0};
    std::shared_ptr<PointCloud> raw_cloud_ {nullptr};
    std::shared_ptr<PointCloud> filtered_cloud_ {nullptr};

};

struct KeyFrame{
    int key_frame_id_{-1};
    double timestamp_{0.0};
    std::string saved_frame_path_;
    bool valid_rtk_{false};
    Vec3 rtk_pos_;
    Se3 lio_pose_;
    double rtk_align_yaw_{0.0};
};


#endif