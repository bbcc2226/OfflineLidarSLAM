#include "ConfigLoader.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace {

template<typename T>
T getOrDefault(const YAML::Node& node, const std::string& key, const T& default_val) {
    if (node[key]) {
        return node[key].as<T>();
    }
    return default_val;
}

} // namespace

Config ConfigLoader::Load(const std::string& file_path) {
    Config cfg;

    YAML::Node root;
    try {
        root = YAML::LoadFile(file_path);
    } catch (const std::exception& e) {
        std::cerr << "[ConfigLoader] Failed to load YAML: " << e.what() << std::endl;
        std::cerr << "[ConfigLoader] Using default config\n";
        return cfg;
    }

    // -------- ESKF --------
    if (root["ESKF"]) {
        auto eskf = root["ESKF"];
        cfg.ESKF_.process_vel_noise = getOrDefault(eskf, "process_vel_noise", cfg.ESKF_.process_vel_noise);
        cfg.ESKF_.process_rot_noise = getOrDefault(eskf, "process_rot_noise", cfg.ESKF_.process_rot_noise);
        cfg.ESKF_.process_ba_noise = getOrDefault(eskf, "process_ba_noise", cfg.ESKF_.process_ba_noise);
        cfg.ESKF_.process_bg_noise = getOrDefault(eskf, "process_bg_noise", cfg.ESKF_.process_bg_noise);
        cfg.ESKF_.measurement_pos_noise = getOrDefault(eskf, "measurement_pos_noise", cfg.ESKF_.measurement_pos_noise);
        cfg.ESKF_.measurement_rot_noise = getOrDefault(eskf, "measurement_rot_noise", cfg.ESKF_.measurement_rot_noise);
    }


    // -------- FrontEnd --------
    if (root["FrontEnd"]) {
        auto front_end = root["FrontEnd"];
        cfg.FrontEnd_.lidar_buffer_upper_capacity = getOrDefault(front_end, "lidar_buffer_upper_capacity", cfg.FrontEnd_.lidar_buffer_upper_capacity);
        cfg.FrontEnd_.imu_buffer_upper_capacity = getOrDefault(front_end, "imu_buffer_upper_capacity", cfg.FrontEnd_.imu_buffer_upper_capacity);
        cfg.FrontEnd_.gps_buffer_upper_capacity = getOrDefault(front_end, "gps_buffer_upper_capacity", cfg.FrontEnd_.gps_buffer_upper_capacity);
        cfg.FrontEnd_.lidar_buffer_lower_capacity = getOrDefault(front_end, "lidar_buffer_lower_capacity", cfg.FrontEnd_.lidar_buffer_lower_capacity);
        cfg.FrontEnd_.imu_buffer_lower_capacity = getOrDefault(front_end, "imu_buffer_lower_capacity", cfg.FrontEnd_.imu_buffer_lower_capacity);
        cfg.FrontEnd_.gps_buffer_lower_capacity = getOrDefault(front_end, "gps_buffer_lower_capacity", cfg.FrontEnd_.gps_buffer_lower_capacity);
        cfg.FrontEnd_.lio_dir_path = getOrDefault(front_end, "lio_dir_path", cfg.FrontEnd_.lio_dir_path);
    }

    if(root["GeoConverter"]){
        auto geo_converter = root["GeoConverter"];
        cfg.GeoConverter_.a = getOrDefault(geo_converter, "a", cfg.GeoConverter_.a);
        cfg.GeoConverter_.f = getOrDefault(geo_converter, "f", cfg.GeoConverter_.f);
        cfg.GeoConverter_.b = getOrDefault(geo_converter, "b", cfg.GeoConverter_.b);
        cfg.GeoConverter_.e2 = getOrDefault(geo_converter, "e2", cfg.GeoConverter_.e2);
    }

    if(root["LidarOdometry"]){
        auto lidar_odometry = root["LidarOdometry"];
        cfg.LidarOdometry_.voxel_resolution = getOrDefault(lidar_odometry, "voxel_resolution", cfg.LidarOdometry_.voxel_resolution);
        cfg.LidarOdometry_.ndt_resolution = getOrDefault(lidar_odometry, "ndt_resolution", cfg.LidarOdometry_.ndt_resolution);
        cfg.LidarOdometry_.lidar_x_range = getOrDefault(lidar_odometry, "lidar_x_range", cfg.LidarOdometry_.lidar_x_range);
        cfg.LidarOdometry_.lidar_y_range = getOrDefault(lidar_odometry, "lidar_y_range", cfg.LidarOdometry_.lidar_y_range);
        cfg.LidarOdometry_.lidar_z_range = getOrDefault(lidar_odometry, "lidar_z_range", cfg.LidarOdometry_.lidar_z_range);
    }

    if(root["DataLoader"]){
        auto data_loader = root["DataLoader"];
        cfg.DataLoader_.remove_groud = getOrDefault(data_loader, "remove_groud", cfg.DataLoader_.remove_groud);
        cfg.DataLoader_.kitti_groud_height = getOrDefault(data_loader, "kitti_groud_height", cfg.DataLoader_.kitti_groud_height);
        cfg.DataLoader_.ros_bag_path = getOrDefault(data_loader, "ros_bag_path", cfg.DataLoader_.ros_bag_path);
    }

    if(root["General"]){
        auto general = root["General"];
        cfg.General_.save_lo_frame = getOrDefault(general, "save_lo_frame", cfg.General_.save_lo_frame);
        cfg.General_.save_lio_frame = getOrDefault(general, "save_lio_frame", cfg.General_.save_lio_frame);
        cfg.General_.filter_saved_cloud = getOrDefault(general, "filter_saved_cloud", cfg.General_.filter_saved_cloud);
        cfg.General_.map_voxel_resolution = getOrDefault(general, "map_voxel_resolution", cfg.General_.map_voxel_resolution);
        cfg.General_.num_pts_threshold_for_viz = getOrDefault(general, "num_pts_threshold_for_viz", cfg.General_.num_pts_threshold_for_viz);

    }
    if(root["Optimizer"]){
        auto optimizer = root["Optimizer"];
        cfg.Optimizer_.gps_edge_weight = getOrDefault(optimizer, "gps_edge_weight", cfg.Optimizer_.gps_edge_weight);
        cfg.Optimizer_.gps_relative_edge_weight = getOrDefault(optimizer, "gps_relative_edge_weight", cfg.Optimizer_.gps_relative_edge_weight);
        cfg.Optimizer_.lio_edge_weight = getOrDefault(optimizer, "lio_edge_weight", cfg.Optimizer_.lio_edge_weight);
        cfg.Optimizer_.local_optimization_widnow_size = getOrDefault(optimizer, "local_optimization_widnow_size", cfg.Optimizer_.local_optimization_widnow_size);
        cfg.Optimizer_.min_keyframe_num_for_optimization = getOrDefault(optimizer, "min_keyframe_num_for_optimization", cfg.Optimizer_.min_keyframe_num_for_optimization);
        cfg.Optimizer_.iterations = getOrDefault(optimizer, "iterations", cfg.Optimizer_.iterations);
    }
    if(root["LoopClosure"]){
        auto loop_closure = root["LoopClosure"];
        cfg.LoopClosure_.loop_closure_search_radius = getOrDefault(loop_closure, "loop_closure_search_radius", cfg.LoopClosure_.loop_closure_search_radius);
        cfg.LoopClosure_.loop_closure_min_keyframe_gap = getOrDefault(loop_closure, "loop_closure_min_keyframe_gap", cfg.LoopClosure_.loop_closure_min_keyframe_gap);
        cfg.LoopClosure_.loop_closure_fitness_score_threshold = getOrDefault(loop_closure, "loop_closure_fitness_score_threshold", cfg.LoopClosure_.loop_closure_fitness_score_threshold);
        cfg.LoopClosure_.top_k_loop_closure_candidates = getOrDefault(loop_closure, "top_k_loop_closure_candidates", cfg.LoopClosure_.top_k_loop_closure_candidates);
        cfg.LoopClosure_.skip_count_for_loop_closure_detection = getOrDefault(loop_closure, "skip_count_for_loop_closure_detection", cfg.LoopClosure_.skip_count_for_loop_closure_detection);
    }
    return cfg;
}