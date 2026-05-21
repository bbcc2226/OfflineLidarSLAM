#include "ScanMatching.hpp"

#include <cmath>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

std::shared_ptr<PointCloud> CoarseToFineRegistration::CropTargetCloud(
    const std::shared_ptr<PointCloud>& fine_target_cloud_world,
    const Se3& coarse_pose) const {

    auto cropped_cloud = std::make_shared<PointCloud>();

    if (!fine_target_cloud_world) {
        return cropped_cloud;
    }

    const Eigen::Vector3d center = coarse_pose.translation();
    const double radius_sq = gicp_crop_radius_ * gicp_crop_radius_;
    cropped_cloud->pt_list_.reserve(fine_target_cloud_world->pt_list_.size());

    for (const auto& pt : fine_target_cloud_world->pt_list_) {
        const Eigen::Vector3d target_pt(pt[0], pt[1], pt[2]);
        if ((target_pt - center).squaredNorm() <= radius_sq) {
            cropped_cloud->pt_list_.push_back(pt);
        }
    }

    return cropped_cloud;
}

std::shared_ptr<CoarseToFineRegistration::PCLCloud>
CoarseToFineRegistration::ToPCLCloud(
    const std::shared_ptr<PointCloud>& cloud) {

    auto pcl_cloud = std::make_shared<PCLCloud>();

    if (!cloud) {
        return pcl_cloud;
    }

    pcl_cloud->reserve(cloud->pt_list_.size());

    for (const auto& pt : cloud->pt_list_) {
        pcl_cloud->emplace_back(
            static_cast<float>(pt[0]),
            static_cast<float>(pt[1]),
            static_cast<float>(pt[2]));
    }

    return pcl_cloud;
}

Se3 CoarseToFineRegistration::Align(
    const std::shared_ptr<PointCloud>& input_scan_ptr,
    NDT_INC& ndt_map,
    const std::shared_ptr<PointCloud>& fine_target_cloud_world,
    const Se3& init_pose,
    int ndt_max_iter) const {

    const Se3 coarse_pose =
        ndt_map.Align(input_scan_ptr,
                      init_pose,
                      ndt_max_iter);

    const double coarse_fitness =
        ndt_map.ComputeFitnessScore(input_scan_ptr, coarse_pose);

    if (!std::isfinite(coarse_fitness) ||
        coarse_fitness > max_ndt_fitness_for_gicp_) {
        return coarse_pose;
    }

    if (!fine_target_cloud_world ||
        fine_target_cloud_world->pt_list_.size() < 30) {
        return coarse_pose;
    }

    auto cropped_target_cloud = CropTargetCloud(
        fine_target_cloud_world,
        coarse_pose);

    if (!cropped_target_cloud ||
        cropped_target_cloud->pt_list_.size() < 30) {
        return coarse_pose;
    }

    return RefineWithGICP(
        input_scan_ptr,
        cropped_target_cloud,
        coarse_pose);
}

Se3 CoarseToFineRegistration::RefineWithGICP(
    const std::shared_ptr<PointCloud>& input_scan_ptr,
    const std::shared_ptr<PointCloud>& fine_target_cloud_world,
    const Se3& coarse_pose) const {

    auto source = ToPCLCloud(input_scan_ptr);
    auto target = ToPCLCloud(fine_target_cloud_world);

    if (!source || !target ||
        source->empty() || target->empty()) {
        return coarse_pose;
    }

    pcl::GeneralizedIterativeClosestPoint<
        pcl::PointXYZ,
        pcl::PointXYZ> gicp;

    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    gicp.setMaximumIterations(gicp_max_iter_);

    gicp.setMaxCorrespondenceDistance(
        gicp_max_correspondence_dist_);

    gicp.setTransformationEpsilon(
        gicp_transform_epsilon_);

    gicp.setEuclideanFitnessEpsilon(
        gicp_fitness_epsilon_);

    PCLCloud aligned_output;

    const Eigen::Matrix4f coarse_guess =
        coarse_pose.matrix().cast<float>();

    gicp.align(aligned_output, coarse_guess);

    if (!gicp.hasConverged()) {
        return coarse_pose;
    }

    Eigen::Matrix4d final_transform =
        gicp.getFinalTransformation().cast<double>();

    Eigen::Matrix3d rotation =
        final_transform.block<3, 3>(0, 0);

    const Eigen::Vector3d translation =
        final_transform.block<3, 1>(0, 3);

    rotation =
        Sophus::SO3d::fitToSO3(rotation).matrix();

    return Se3(rotation, translation);
}