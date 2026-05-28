#ifndef __COARSE_TO_FINE_REGISTRATION__
#define __COARSE_TO_FINE_REGISTRATION__

#include "DataType.hpp"
#include "NDT_INC.hpp"
#include "VoxelFilter.hpp"

#include <deque>
#include <memory>

namespace pcl {
template <typename PointT>
class PointCloud;

struct PointXYZ;
} // namespace pcl

class CoarseToFineRegistration {
public:
    static constexpr size_t kFineTargetWindowSize = 30;

    CoarseToFineRegistration(double gicp_max_correspondence_dist,
                             int gicp_max_iter,
                             double gicp_transform_epsilon,
                             double gicp_fitness_epsilon)
        : gicp_max_correspondence_dist_(gicp_max_correspondence_dist),
          gicp_max_iter_(gicp_max_iter),
          gicp_transform_epsilon_(gicp_transform_epsilon),
          gicp_fitness_epsilon_(gicp_fitness_epsilon) {}

    void AddTargetFrame(const std::shared_ptr<PointCloud>& cloud);

    Se3 Align(const std::shared_ptr<PointCloud>& input_scan_ptr,
              NDT_INC& ndt_map,
              const Se3& init_pose,
              int ndt_max_iter = 10);

private:
    using PCLCloud = pcl::PointCloud<pcl::PointXYZ>;

    void RebuildTargetMap();

    std::shared_ptr<PointCloud> CropTargetCloud(
        const Se3& coarse_pose) const;

    static std::shared_ptr<PCLCloud> ToPCLCloud(
        const std::shared_ptr<PointCloud>& cloud);

    Se3 RefineWithGICP(
        const std::shared_ptr<PointCloud>& input_scan_ptr,
        const std::shared_ptr<PointCloud>& fine_target_cloud_world,
        const Se3& coarse_pose) const;

    double gicp_max_correspondence_dist_;
    int gicp_max_iter_;
    double gicp_transform_epsilon_;
    double gicp_fitness_epsilon_;
    double gicp_crop_radius_ {15.0};
    double max_ndt_fitness_for_gicp_ {5.0};
    std::shared_ptr<PointCloud> fine_target_cloud_world_ {std::make_shared<PointCloud>()};
    std::deque<std::shared_ptr<PointCloud>> fine_target_keyframes_world_;
};

#endif