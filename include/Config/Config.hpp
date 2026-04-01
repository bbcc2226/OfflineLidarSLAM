#pragma once

#include <Eigen/Dense>
#include <string>

struct ESKFConfig{
    double process_vel_noise = 0.1;
    double process_rot_noise = 0.1;
    double process_ba_noise = 1.0e-7;
    double process_bg_noise = 1.0e-7;
    double measurement_pos_noise = 1.0e-5;
    double measurement_rot_noise = 1.0e-5;
};

struct FrontEndConfig{
    int lidar_buffer_upper_capacity = 100;
    int imu_buffer_upper_capacity = 2000;
    int gps_buffer_upper_capacity = 2000;

    int lidar_buffer_lower_capacity = 10;
    int imu_buffer_lower_capacity = 200;
    int gps_buffer_lower_capacity = 200;

    std::string lio_dir_path = "./LIO_results";
};

struct GeoConverterConfig{
    double a = 6378137.0;                       // WGS84 semi-major axis
    double f = 1.0 / 298.257223563;             // flattening
    double b = a * (1 - f);
    double e2 = 1 - (b * b) / (a * a);
};

struct LidarOdometryConfig{
    double voxel_resolution = 0.7;
    double ndt_resolution = 4.0;
};

struct DataLoaderConfig{
    bool remove_groud = true;
    double kitti_groud_height = -1.0;
    //std::string ros_bag_path = "/root/ros2_ws/2011_09_26_drive_0035_extract_bag";
    std::string ros_bag_path = "/root/ros2_ws/2011_10_03_drive_0027_bag";
    
};

struct OptimizerConfig{
    double gps_edge_weight = 1.0;
    double yaw_edge_weight = 1.0;
    double lio_edge_weight = 100.0;
    int local_optimization_widnow_size = 30;
    int min_keyframe_num_for_optimization = 10;
    int iterations = 20;
};

struct GeneralConfig{
    double map_voxel_resolution = 0.5;
    bool save_lo_frame = false;
    bool save_lio_frame = true;
    bool filter_saved_cloud = true;
};

struct Config{
    ESKFConfig ESKF_;
    FrontEndConfig FrontEnd_;
    GeoConverterConfig GeoConverter_;
    LidarOdometryConfig LidarOdometry_;
    DataLoaderConfig DataLoader_;
    OptimizerConfig Optimizer_;
    GeneralConfig General_;
};



