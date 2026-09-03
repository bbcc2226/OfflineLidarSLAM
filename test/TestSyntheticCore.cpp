#include <gtest/gtest.h>

#include "ConfigLoader.hpp"
#include "GeoConverter.hpp"
#include "VoxelFilter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

const Point3D* FindPointNearX(const PointCloud& cloud, double x) {
    const auto it = std::find_if(
        cloud.pt_list_.begin(), cloud.pt_list_.end(),
        [x](const Point3D& point) { return std::abs(point[0] - x) < 1e-12; });
    return it == cloud.pt_list_.end() ? nullptr : &*it;
}

class TemporaryYaml {
public:
    explicit TemporaryYaml(const std::string& contents) {
        const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path() /
                ("offline_lidar_slam_config_" + std::to_string(suffix) + ".yaml");
        std::ofstream output(path_);
        output << contents;
        if (!output) {
            throw std::runtime_error("failed to create temporary YAML file");
        }
    }

    ~TemporaryYaml() {
        std::error_code error;
        std::filesystem::remove(path_, error);
    }

    const std::filesystem::path& Path() const { return path_; }

private:
    std::filesystem::path path_;
};

}  // namespace

TEST(VoxelFilterSynthetic, ReplacesEachVoxelWithItsCentroid) {
    auto input = std::make_shared<PointCloud>();
    input->pt_list_ = {
        Point3D{0.1, 0.2, 0.3},
        Point3D{0.5, 0.4, 0.1},
        Point3D{1.2, 0.1, 0.1},
    };

    VoxelFilter filter(1.0);
    const auto output = filter.Downsample(input, false);

    ASSERT_EQ(output->pt_list_.size(), 2U);
    const Point3D* first_voxel = FindPointNearX(*output, 0.3);
    ASSERT_NE(first_voxel, nullptr);
    EXPECT_NEAR((*first_voxel)[1], 0.3, 1e-12);
    EXPECT_NEAR((*first_voxel)[2], 0.2, 1e-12);

    const Point3D* second_voxel = FindPointNearX(*output, 1.2);
    ASSERT_NE(second_voxel, nullptr);
    EXPECT_NEAR((*second_voxel)[1], 0.1, 1e-12);
    EXPECT_NEAR((*second_voxel)[2], 0.1, 1e-12);
}

TEST(VoxelFilterSynthetic, SeparatesPointsAcrossZeroBoundary) {
    auto input = std::make_shared<PointCloud>();
    input->pt_list_ = {Point3D{-0.1, 0.0, 0.0}, Point3D{0.1, 0.0, 0.0}};

    VoxelFilter filter(1.0);
    const auto output = filter.Downsample(input, false);

    EXPECT_EQ(output->pt_list_.size(), 2U);
}

TEST(GeoConverterSynthetic, FirstFixDefinesTheEnuOrigin) {
    GeoConverter converter;
    const Vec3 origin = converter.Geo2ENU(Vec3(37.0, -122.0, 15.0));

    EXPECT_TRUE(origin.isApprox(Vec3::Zero(), 1e-12));
}

TEST(GeoConverterSynthetic, ConvertsKnownEquatorialOffsets) {
    GeoConverter east_converter;
    ASSERT_TRUE(east_converter.Geo2ENU(Vec3(0.0, 0.0, 0.0)).isZero(1e-12));
    const Vec3 east = east_converter.Geo2ENU(Vec3(0.0, 0.001, 0.0));

    EXPECT_NEAR(east.x(), 111.31949, 1e-3);
    EXPECT_NEAR(east.y(), 0.0, 1e-6);
    EXPECT_NEAR(east.z(), -0.000972, 1e-5);

    GeoConverter altitude_converter;
    ASSERT_TRUE(altitude_converter.Geo2ENU(Vec3(0.0, 0.0, 5.0)).isZero(1e-12));
    const Vec3 above = altitude_converter.Geo2ENU(Vec3(0.0, 0.0, 15.0));
    EXPECT_NEAR(above.x(), 0.0, 1e-12);
    EXPECT_NEAR(above.y(), 0.0, 1e-12);
    EXPECT_NEAR(above.z(), 10.0, 1e-9);
}

TEST(ConfigLoaderSynthetic, LoadsDataSourceOverrides) {
    const TemporaryYaml yaml(
        "DataLoader:\n"
        "  ros_bag_path: /tmp/example_bag\n"
        "  storage_id: mcap\n"
        "  lidar_topic: /sensors/lidar\n"
        "  imu_topic: /sensors/imu\n"
        "  gps_topic: /sensors/gps\n");

    const Config config = ConfigLoader::Load(yaml.Path().string());

    EXPECT_EQ(config.DataLoader_.ros_bag_path, "/tmp/example_bag");
    EXPECT_EQ(config.DataLoader_.storage_id, "mcap");
    EXPECT_EQ(config.DataLoader_.lidar_topic, "/sensors/lidar");
    EXPECT_EQ(config.DataLoader_.imu_topic, "/sensors/imu");
    EXPECT_EQ(config.DataLoader_.gps_topic, "/sensors/gps");
    EXPECT_DOUBLE_EQ(config.LidarOdometry_.voxel_resolution, 0.7);
}
