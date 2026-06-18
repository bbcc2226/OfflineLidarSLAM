#include "NDT_INC.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <sstream>
#include <iomanip>
#include "DataType.hpp"  // make sure your PointCloud/Point3D structs are defined
#include "Common.hpp"
#include "TicToc.hpp"


// TEST(NDT_INC_Test, RealTest)
// {

//     std::shared_ptr<PointCloud> source_cloud_ptr = LoadKittiBin("lidar1.bin");
//     std::shared_ptr<PointCloud> target_cloud_ptr = LoadKittiBin("lidar2.bin");

//     TicToc timer;

//     timer.tic();
//     NDT_INC ndt_map(5);
//     if(!ndt_map.IsIntialized()){
//         ndt_map.AddCloud(ApplyDownSampleFilter(source_cloud_ptr));
//     }
//     Sophus::SE3d est = ndt_map.Align(ApplyDownSampleFilter(target_cloud_ptr),Sophus::SE3d());

//     timer.toc("NDT processing time is: ");
//     std::cout << "Estimated:" << est.matrix() << std::endl;

//     SaveCloud(source_cloud_ptr,GenerateFramePath("./",0));
//     SaveCloud(ApplyTransform(est,target_cloud_ptr),GenerateFramePath("./",1));


//     // auto cloud_ref = ToPCL(o_source_cloud_ptr);                 // reference scan
//     // //auto cloud_aligned = ToPCL(source_cloud_ptr);                 // reference scan

//     // auto cloud_aligned = ToPCL(ndt_map.ApplyTransform(est,o_target_cloud_ptr));    // aligned scan

//     // pcl::io::savePLYFile("output_ref.ply", *cloud_ref);
//     // pcl::io::savePLYFile("output_align.ply", *cloud_aligned);

// }

// Helper: replace the optimized_pose in a keyframe block string
static std::string ReplaceOptimizedPose(const std::string& block, const Sophus::SE3d& new_pose) {
    Eigen::Vector3d t = new_pose.translation();
    Eigen::Quaterniond q = new_pose.unit_quaternion();  // x, y, z, w

    // Find optimized_pose section boundaries
    auto opt_start = block.find("\"optimized_pose\"");
    if (opt_start == std::string::npos) return block;
    auto brace_open = block.find('{', opt_start);
    auto brace_close = block.find('}', brace_open);
    if (brace_open == std::string::npos || brace_close == std::string::npos) return block;

    std::ostringstream replacement;
    replacement << std::fixed << std::setprecision(10);
    replacement << "\"optimized_pose\": {\n"
                << "      \"translation\": [" << t[0] << ", " << t[1] << ", " << t[2] << "],\n"
                << "      \"quaternion_xyzw\": [" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << "]\n"
                << "    }";

    return block.substr(0, opt_start) + replacement.str() + block.substr(brace_close + 1);
}

// Helper: write all blocks back to the jsonl file
static bool WriteKeyframeBlocks(const std::string& filepath, const std::vector<std::string>& blocks) {
    std::ofstream fout(filepath);
    if (!fout.is_open()) return false;
    for (const auto& b : blocks) fout << b;
    return true;
}

// Helper: read all keyframe JSON blocks from a jsonl file (multi-line entries)
static std::vector<std::string> ReadKeyframeBlocks(const std::string& filepath) {
    std::ifstream fin(filepath);
    std::vector<std::string> blocks;
    if (!fin.is_open()) return blocks;
    std::string block, line;
    int depth = 0;
    while (std::getline(fin, line)) {
        for (char c : line) {
            if (c == '{') depth++;
            else if (c == '}') depth--;
        }
        block += line + "\n";
        if (depth == 0 && !block.empty() && block.find('{') != std::string::npos) {
            blocks.push_back(block);
            block.clear();
        }
    }
    return blocks;
}

// Helper: find keyframe block by key_frame_id
static std::string FindKeyframeBlock(const std::vector<std::string>& blocks, int target_id) {
    for (const auto& b : blocks) {
        auto pos = b.find("\"key_frame_id\"");
        if (pos == std::string::npos) continue;
        auto colon = b.find(':', pos);
        auto comma = b.find(',', colon);
        int id = std::stoi(b.substr(colon + 1, comma - colon - 1));
        if (id == target_id) return b;
    }
    return "";
}

// Helper: extract string value from JSON block
static std::string ExtractString(const std::string& block, const std::string& key) {
    auto pos = block.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    auto colon = block.find(':', pos);
    auto q1 = block.find('"', colon + 1);
    auto q2 = block.find('"', q1 + 1);
    return block.substr(q1 + 1, q2 - q1 - 1);
}

// Helper: extract array of doubles from JSON block after a key
static std::vector<double> ExtractArray(const std::string& block, const std::string& key) {
    auto pos = block.find("\"" + key + "\"");
    if (pos == std::string::npos) return {};
    auto lb = block.find('[', pos);
    auto rb = block.find(']', lb);
    std::string arr = block.substr(lb + 1, rb - lb - 1);
    std::vector<double> vals;
    std::stringstream ss(arr);
    std::string item;
    while (std::getline(ss, item, ',')) vals.push_back(std::stod(item));
    return vals;
}

// Helper: extract optimized_pose as SE3
static Sophus::SE3d ExtractOptimizedPose(const std::string& block) {
    auto opt_pos = block.find("\"optimized_pose\"");
    if (opt_pos == std::string::npos) return Sophus::SE3d();
    std::string sub = block.substr(opt_pos);
    auto t = ExtractArray(sub, "translation");
    auto q = ExtractArray(sub, "quaternion_xyzw");
    if (t.size() < 3 || q.size() < 4) return Sophus::SE3d();
    Eigen::Vector3d trans(t[0], t[1], t[2]);
    Eigen::Quaterniond quat(q[3], q[0], q[1], q[2]);  // w, x, y, z
    return Sophus::SE3d(quat.normalized(), trans);
}

// Helper: extract rtk-based world pose as SE3
//   translation = R_z(rtk_align_yaw) * rtk_pos_enu  (converts ENU -> LIO frame)
//   rotation    = from optimized_pose (RTK only gives position + coarse yaw, not full 6DOF)
static bool ExtractRTKPose(const std::string& block, Sophus::SE3d& out_pose) {
    // Check valid_rtk
    auto pos = block.find("\"valid_rtk\"");
    if (pos == std::string::npos) return false;
    auto colon = block.find(':', pos);
    auto eol   = block.find('\n', colon);
    std::string val = block.substr(colon + 1, eol - colon - 1);
    val.erase(remove_if(val.begin(), val.end(), ::isspace), val.end());
    val.erase(remove(val.begin(), val.end(), ','), val.end());
    if (val != "true") return false;

    auto t = ExtractArray(block, "rtk_pos");
    if (t.size() < 3) return false;

    // rtk_align_yaw: rotation to convert ENU -> LIO frame
    auto yaw_pos = block.find("\"rtk_align_yaw\"");
    if (yaw_pos == std::string::npos) return false;
    auto yaw_colon = block.find(':', yaw_pos);
    auto yaw_comma = block.find_first_of(",\n", yaw_colon);
    double yaw = std::stod(block.substr(yaw_colon + 1, yaw_comma - yaw_colon - 1));

    // Transform ENU position into LIO frame
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d rtk_pos_enu(t[0], t[1], t[2]);
    Eigen::Vector3d trans_lio = R_yaw * rtk_pos_enu;
    std::cout << "Extracted RTK ENU position: " << trans_lio.transpose() << ", yaw: " << yaw << " rad\n";
    // Rotation from optimized_pose (RTK provides position, not full 6DOF orientation)
    Sophus::SE3d opt_pose = ExtractOptimizedPose(block);
    out_pose = Sophus::SE3d(opt_pose.rotationMatrix(), trans_lio);
    return true;
}

TEST(NDT_INC_Test, KeyframeNDTMatching)
{
    // ===== CHANGE THESE IDS TO SELECT REFERENCE / QUERY KEYFRAMES =====
    const int kf_id1 = 95;
    const int kf_id2 = 241;
    const int map_half_window = 10;
    // ==================================================================
    VoxelFilter filter(0.7);
    const std::string jsonl_path = "/root/ros2_ws/build/offline_lidar_slam/key_frames.jsonl";
    const std::string cloud_base = "/root/ros2_ws/build/offline_lidar_slam/";

    auto blocks = ReadKeyframeBlocks(jsonl_path);
    ASSERT_FALSE(blocks.empty()) << "No keyframe blocks found in " << jsonl_path;

    std::string block1 = FindKeyframeBlock(blocks, kf_id1);
    std::string block2 = FindKeyframeBlock(blocks, kf_id2);
    ASSERT_FALSE(block1.empty()) << "Keyframe id=" << kf_id1 << " not found";
    ASSERT_FALSE(block2.empty()) << "Keyframe id=" << kf_id2 << " not found";

    std::string rel_path1 = ExtractString(block1, "saved_frame_path");
    std::string rel_path2 = ExtractString(block2, "saved_frame_path");
    // paths look like "./LIO_results/cloud_000024.ply" — strip leading "./"
    if (rel_path1.substr(0, 2) == "./") rel_path1 = rel_path1.substr(2);
    if (rel_path2.substr(0, 2) == "./") rel_path2 = rel_path2.substr(2);

    std::string full_path1 = cloud_base + rel_path1;
    std::string full_path2 = cloud_base + rel_path2;
    std::cout << "Loading reference keyframe " << kf_id1 << ": " << full_path1 << std::endl;
    std::cout << "Loading query keyframe " << kf_id2 << ": " << full_path2 << std::endl;

    std::shared_ptr<PointCloud> cloud2 = std::make_shared<PointCloud>();
    ASSERT_TRUE(LoadPLY(full_path2, cloud2)) << "Failed to load PLY: " << full_path2;
    cloud2 = filter.Downsample(cloud2);
    std::cout << "Query cloud size: " << cloud2->pt_list_.size() << std::endl;

    Sophus::SE3d pose1 = ExtractOptimizedPose(block1);
    Sophus::SE3d pose2 = ExtractOptimizedPose(block2);
    std::cout << "Pose1 translation: " << pose1.translation().transpose() << std::endl;
    std::cout << "Pose2 translation: " << pose2.translation().transpose() << std::endl;

    std::shared_ptr<PointCloud> map_cloud = std::make_shared<PointCloud>();
    int map_frame_count = 0;
    for (int map_kf_id = kf_id1 - map_half_window; map_kf_id <= kf_id1 + map_half_window; ++map_kf_id) {
        std::string map_block = FindKeyframeBlock(blocks, map_kf_id);
        if (map_block.empty()) {
            std::cout << "Skipping missing keyframe " << map_kf_id << " while building map" << std::endl;
            continue;
        }

        std::string map_rel_path = ExtractString(map_block, "saved_frame_path");
        if (map_rel_path.substr(0, 2) == "./") map_rel_path = map_rel_path.substr(2);

        std::shared_ptr<PointCloud> map_frame_cloud = std::make_shared<PointCloud>();
        const std::string map_full_path = cloud_base + map_rel_path;
        ASSERT_TRUE(LoadPLY(map_full_path, map_frame_cloud)) << "Failed to load map-frame PLY: " << map_full_path;

        map_frame_cloud = filter.Downsample(map_frame_cloud);
        Sophus::SE3d map_frame_pose = ExtractOptimizedPose(map_block);
        Sophus::SE3d map_frame_in_ref = pose1.inverse() * map_frame_pose;
        InplaceApplyTransform(map_frame_in_ref, map_frame_cloud);
        map_cloud->pt_list_.insert(map_cloud->pt_list_.end(),
                                   map_frame_cloud->pt_list_.begin(),
                                   map_frame_cloud->pt_list_.end());
        ++map_frame_count;
    }

    ASSERT_GT(map_frame_count, 0) << "No map frames found around keyframe " << kf_id1;
    ASSERT_FALSE(map_cloud->pt_list_.empty()) << "Map cloud is empty";
    std::cout << "Map built from " << map_frame_count << " frame(s), total points: "
              << map_cloud->pt_list_.size() << std::endl;

    // Initial guess: prefer RTK-based pose when valid_rtk is true for both keyframes
    Sophus::SE3d rtk_pose1, rtk_pose2;
    bool rtk1_valid = ExtractRTKPose(block1, rtk_pose1);
    bool rtk2_valid = ExtractRTKPose(block2, rtk_pose2);
    Sophus::SE3d init_guess;
    if (rtk1_valid && rtk2_valid && false) {
        init_guess = rtk_pose1.inverse() * rtk_pose2;
        std::cout << "Using RTK-based initial guess:\n" << init_guess.matrix() << std::endl;
    } else {
        init_guess = pose1.inverse() * pose2;
        std::cout << "Using optimized_pose difference as initial guess:\n" << init_guess.matrix() << std::endl;
    }

    // NDT matching: build a local target map around kf_id1 in the kf_id1 frame,
    // then align the query scan with init_guess = pose1^-1 * pose2.
    // est = refined pose of kf_id2 in the kf_id1 local frame; new absolute pose = pose1 * est.
    TicToc timer;
    timer.tic();
    NDT_INC ndt_map(6.0, 6.0, 3.0);
    ndt_map.AddCloud(map_cloud);
    Sophus::SE3d est = ndt_map.Align(cloud2, init_guess, 40);
    timer.toc("NDT processing time: ");

    std::cout << "NDT estimated transform:\n" << est.matrix() << std::endl;
    ASSERT_TRUE(est.matrix().allFinite());

    // Compute fitness score before and after alignment
    double score_init  = ndt_map.ComputeFitnessScore(cloud2, init_guess);
    double score_after = ndt_map.ComputeFitnessScore(cloud2, est);
    std::cout << "Fitness score (initial guess): " << score_init  << std::endl;
    std::cout << "Fitness score (after NDT):     " << score_after << std::endl;

    // Updated absolute world pose of kf_id2: pose1 * est
    // (est maps local cloud2 into local cloud1 frame; pose1 brings that into world frame)
    Sophus::SE3d new_pose2 = pose1 * est;
    std::cout << "Updated world pose for kf_id2=" << kf_id2 << ":\n" << new_pose2.matrix() << std::endl;

    // Write updated optimized_pose back to the jsonl file
    std::string updated_block2 = ReplaceOptimizedPose(block2, new_pose2);
    for (auto& b : blocks) {
        auto pos = b.find("\"key_frame_id\"");
        if (pos == std::string::npos) continue;
        auto colon = b.find(':', pos);
        auto comma = b.find(',', colon);
        int id = std::stoi(b.substr(colon + 1, comma - colon - 1));
        if (id == kf_id2) { b = updated_block2; break; }
    }
    bool write_ok = WriteKeyframeBlocks(jsonl_path, blocks);
    ASSERT_TRUE(write_ok) << "Failed to write updated jsonl: " << jsonl_path;
    std::cout << "Updated optimized_pose for keyframe " << kf_id2 << " written to " << jsonl_path << std::endl;
}


// TEST(NDT_INC_Test, RealTest2)
// {

//     std::shared_ptr<PointCloud> source_cloud_ptr = LoadKittiBin("lidar.bin");
//     std::shared_ptr<PointCloud> target_cloud_ptr = LoadKittiBin("lidar1.bin");
//     std::shared_ptr<PointCloud> next_cloud_ptr = LoadKittiBin("lidar2.bin");
    
//     TicToc timer;

//     timer.tic();
//     NDT_INC ndt_map(5);
//     if(!ndt_map.IsIntialized()){
//         ndt_map.AddCloud(ApplyDownSampleFilter(source_cloud_ptr));
//     }
//     std::shared_ptr<PointCloud> filter_target_cloud_ptr = ApplyDownSampleFilter(target_cloud_ptr);
//     Sophus::SE3d est = ndt_map.Align(filter_target_cloud_ptr,Sophus::SE3d());
//     InplaceApplyTransform(est,filter_target_cloud_ptr);
//     ndt_map.AddCloud(filter_target_cloud_ptr);
//     std::cout << "Estimated:" << est.matrix() << std::endl;

//     Sophus::SE3d new_est = ndt_map.Align(ApplyDownSampleFilter(next_cloud_ptr),est);

//     std::cout << "Estimated:" << new_est.matrix() << std::endl;

//     timer.toc("NDT processing time is: ");

// }