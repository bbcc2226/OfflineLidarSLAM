#include "NDT.hpp"
#include "NDT_INC.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include "DataType.hpp"  // make sure your PointCloud/Point3D structs are defined
#include "Common.hpp"
#include "TicToc.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>



TEST(NDTTest, SimTest)
{

    std::shared_ptr<PointCloud> target_cloud_ptr = ApplyDownSampleFilter(LoadKittiBin("lidar.bin"));
    std::shared_ptr<PointCloud> source_cloud_ptr = std::make_shared<PointCloud>();
    Sophus::SE3d gt(Sophus::SO3d::exp(Eigen::Vector3d(0.1,0.2,0)),Eigen::Vector3d(0.2,0.2,0.1));

    Eigen::Vector3d point;
    for (size_t i = 0; i < target_cloud_ptr->pt_list_.size(); ++i){
        point << target_cloud_ptr->pt_list_[i][0],target_cloud_ptr->pt_list_[i][1],target_cloud_ptr->pt_list_[i][2];
        Eigen::Vector3d trans_pt = gt.inverse() * point;
        source_cloud_ptr->pt_list_.push_back({trans_pt[0],trans_pt[1],trans_pt[2]});
    }
    
    NDTGridMap ndt_map(5,target_cloud_ptr);
    ndt_map.BuildVoxelGrid();

    Sophus::SE3d est = ndt_map.Align(source_cloud_ptr, Sophus::SE3d());


    std::cout << "Estimated:" << est.matrix() << std::endl;
    std::cout << "GT:" << gt.matrix() << std::endl;

}

TEST(NDTTest, RealTest)
{

    std::shared_ptr<PointCloud> target_cloud_ptr = ApplyDownSampleFilter(LoadKittiBin("lidar.bin"));
    std::shared_ptr<PointCloud> source_cloud_ptr = ApplyDownSampleFilter(LoadKittiBin("lidar1.bin"));
    
    TicToc timer;

    timer.tic();
    NDTGridMap ndt_map(5,target_cloud_ptr);
    ndt_map.BuildVoxelGrid();

    Sophus::SE3d est = ndt_map.Align(source_cloud_ptr, Sophus::SE3d());

    timer.toc("NDT processing time is: ");
    std::cout << "Estimated:" << est.matrix() << std::endl;

}


TEST(NDT_INC_Test, RealTest)
{

    std::shared_ptr<PointCloud> source_cloud_ptr = LoadKittiBin("lidar1.bin");
    std::shared_ptr<PointCloud> target_cloud_ptr = LoadKittiBin("lidar2.bin");

    TicToc timer;

    timer.tic();
    NDT_INC ndt_map(5);
    if(!ndt_map.IsIntialized()){
        ndt_map.AddCloud(ApplyDownSampleFilter(source_cloud_ptr));
    }
    Sophus::SE3d est = ndt_map.Align(ApplyDownSampleFilter(target_cloud_ptr),Sophus::SE3d());

    timer.toc("NDT processing time is: ");
    std::cout << "Estimated:" << est.matrix() << std::endl;

    SaveCloud(source_cloud_ptr,GenerateFramePath("./",0));
    SaveCloud(ApplyTransform(est,target_cloud_ptr),GenerateFramePath("./",1));


    // auto cloud_ref = ToPCL(o_source_cloud_ptr);                 // reference scan
    // //auto cloud_aligned = ToPCL(source_cloud_ptr);                 // reference scan

    // auto cloud_aligned = ToPCL(ndt_map.ApplyTransform(est,o_target_cloud_ptr));    // aligned scan

    // pcl::io::savePLYFile("output_ref.ply", *cloud_ref);
    // pcl::io::savePLYFile("output_align.ply", *cloud_aligned);

}


TEST(NDT_INC_Test, RealTest2)
{

    std::shared_ptr<PointCloud> source_cloud_ptr = LoadKittiBin("lidar.bin");
    std::shared_ptr<PointCloud> target_cloud_ptr = LoadKittiBin("lidar1.bin");
    std::shared_ptr<PointCloud> next_cloud_ptr = LoadKittiBin("lidar2.bin");
    
    TicToc timer;

    timer.tic();
    NDT_INC ndt_map(5);
    if(!ndt_map.IsIntialized()){
        ndt_map.AddCloud(ApplyDownSampleFilter(source_cloud_ptr));
    }
    std::shared_ptr<PointCloud> filter_target_cloud_ptr = ApplyDownSampleFilter(target_cloud_ptr);
    Sophus::SE3d est = ndt_map.Align(filter_target_cloud_ptr,Sophus::SE3d());
    InplaceApplyTransform(est,filter_target_cloud_ptr);
    ndt_map.AddCloud(filter_target_cloud_ptr);
    std::cout << "Estimated:" << est.matrix() << std::endl;

    Sophus::SE3d new_est = ndt_map.Align(ApplyDownSampleFilter(next_cloud_ptr),est);

    std::cout << "Estimated:" << new_est.matrix() << std::endl;

    timer.toc("NDT processing time is: ");

}