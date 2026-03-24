#include <gtest/gtest.h>
#include "LidarOdometry.hpp"

#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>

namespace fs = std::filesystem;

std::vector<fs::path> GetFiles(const std::string& folder) {
    std::vector<fs::path> files;

    for (const auto& entry : fs::directory_iterator(folder)) {
        if (entry.is_regular_file()) {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end());
    return files;
}

TEST(LO_Test, TestSequence){
    std::vector<fs::path> files = GetFiles("./01/velodyne");
    LidarOdodmetry LO(5);
    int frame_count_limit = 100;
    for (const auto& f : files) {
        frame_count_limit -= 1;
        if(frame_count_limit <= 0){
            break;
        }
        std::shared_ptr<PointCloud> curr_raw_cloud_ptr = LoadKittiBin(f);
        std::shared_ptr<PointCloud> filtered_cloud_ptr = ApplyDownSampleFilter(curr_raw_cloud_ptr);
        Se3 temp_pose;
        LO.AddCloud(filtered_cloud_ptr,curr_raw_cloud_ptr,temp_pose,true);
        std::cout <<"processing : "<< f << std::endl;
    }
}