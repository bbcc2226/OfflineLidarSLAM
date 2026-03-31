#pragma once

#include <Eigen/Dense>
#include <string>

// namespace Config
// {

//     namespace ESKF
//     {
//         constexpr double process_vel_noise = 0.1;
//         constexpr double process_rot_noise = 0.1;
//         constexpr double process_ba_noise = 1.0e-7;
//         constexpr double process_bg_noise = 1.0e-7;
//         constexpr double measurement_pos_noise = 1.0e-5;
//         constexpr double measurement_rot_noise = 1.0e-5;
//     }

//     namespace FrontEnd{

//         inline const Eigen::Matrix4d T_IMU_LIDAR = (Eigen::Matrix4d() <<
//             9.999976e-01,  7.553071e-04, -2.035826e-03, -8.086759e-01,
//             -7.854027e-04,  9.998898e-01, -1.482298e-02,  3.195559e-01,
//             2.024406e-03,  1.482454e-02,  9.998881e-01, -7.997231e-01,
//             0.0,            0.0,           0.0,           1.0
//         ).finished();

//         inline const Eigen::Matrix4d T_LIDAR_IMU = T_IMU_LIDAR.inverse();

//         constexpr int lidar_buffer_upper_capacity = 100;
//         constexpr int imu_buffer_upper_capacity = 2000;
//         constexpr int gps_buffer_upper_capacity = 2000;

//         constexpr int lidar_buffer_lower_capacity = 10;
//         constexpr int imu_buffer_lower_capacity = 200;
//         constexpr int gps_buffer_lower_capacity = 200;

//         inline const std::string lio_dir_path = "./LIO_results";
//     }

//     namespace GeoConverter
//     {
//     constexpr double a = 6378137.0;                       // WGS84 semi-major axis
//     constexpr double f = 1.0 / 298.257223563;             // flattening
//     constexpr double b = a * (1 - f);
//     constexpr double e2 = 1 - (b * b) / (a * a);
//     }

//     namespace LidarOdometry{
//         constexpr double voxel_resolution = 0.7;
//         constexpr double ndt_resolution = 4.0;
//     }

//     namespace DataLoader{
//         constexpr bool remove_groud = true;
//         constexpr double kitti_groud_height = -1.0;
//         //const std::string ros_bag_path = "/root/ros2_ws/2011_09_26_drive_0035_extract_bag";
//         const std::string ros_bag_path = "/root/ros2_ws/2011_10_03_drive_0027_bag";
        
//     }

//     namespace General{
//         constexpr bool save_lo_frame = false;
//         constexpr bool save_lio_frame = true;
//         constexpr bool filter_saved_cloud = true;
//     }
// }

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
        GeneralConfig General_;
    };



