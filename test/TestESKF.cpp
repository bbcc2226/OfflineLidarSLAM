#include <gtest/gtest.h>

#include "ConfigManager.hpp"
#include "ESKF.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <memory>

namespace {

constexpr double kInitialTimestamp = 100.0;

void LoadTestConfig() {
    ConfigManager::Load(TEST_CONFIG_PATH);
}

Se3 InitialPose() {
    return Se3(
        Eigen::AngleAxisd(0.3, Vec3::UnitZ()).toRotationMatrix(),
        Vec3(1.0, 2.0, 3.0));
}

std::shared_ptr<IMUdata> StationaryImu(double timestamp) {
    const auto& cfg = ConfigManager::Get().ESKF_;
    const double roll = cfg.initial_roll_deg * M_PI / 180.0;
    const double pitch = cfg.initial_pitch_deg * M_PI / 180.0;
    const Eigen::Matrix3d body_to_level =
        (Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
         Eigen::AngleAxisd(roll, Vec3::UnitX())).toRotationMatrix();

    auto imu = std::make_shared<IMUdata>();
    imu->timestamp_ = timestamp;
    imu->acc_ =
        body_to_level.transpose() *
        Vec3(0.0, 0.0, cfg.gravity_magnitude);
    imu->acc_ += Vec3(cfg.initial_acc_bias_x,
                      cfg.initial_acc_bias_y,
                      cfg.initial_acc_bias_z);
    imu->gyro_ = Vec3(cfg.initial_gyro_bias_x,
                      cfg.initial_gyro_bias_y,
                      cfg.initial_gyro_bias_z);
    return imu;
}

}  // namespace

TEST(ESKFTest, InitializesFromFirstLidarPose) {
    LoadTestConfig();
    ESKF eskf;
    const Se3 initial_pose = InitialPose();

    eskf.MeasurementUpdateLidar(initial_pose, kInitialTimestamp);

    const Se3 output = eskf.GetLocalizationOutput();
    EXPECT_TRUE(output.translation().isApprox(initial_pose.translation(), 1e-12));
    EXPECT_NEAR(
        (output.so3().inverse() * initial_pose.so3()).log().norm(),
        0.0,
        1e-12);
}

TEST(ESKFTest, OptionOneGravityKeepsStationaryStateFixed) {
    LoadTestConfig();
    ESKF eskf;
    const Se3 initial_pose = InitialPose();
    eskf.MeasurementUpdateLidar(initial_pose, kInitialTimestamp);

    eskf.Prediction(StationaryImu(kInitialTimestamp + 0.005));

    const Se3 output = eskf.GetLocalizationOutput();
    EXPECT_TRUE(output.translation().isApprox(initial_pose.translation(), 1e-10));
    EXPECT_NEAR(
        (output.so3().inverse() * initial_pose.so3()).log().norm(),
        0.0,
        1e-10);
}

TEST(ESKFTest, SkipsPropagationAcrossLargeImuGap) {
    LoadTestConfig();
    ESKF eskf;
    const Se3 initial_pose = InitialPose();
    eskf.MeasurementUpdateLidar(initial_pose, kInitialTimestamp);

    auto imu = StationaryImu(kInitialTimestamp + 1.0);
    imu->acc_ += Vec3(100.0, 0.0, 0.0);
    imu->gyro_ += Vec3(0.0, 0.0, 10.0);
    eskf.Prediction(imu);

    const Se3 output = eskf.GetLocalizationOutput();
    EXPECT_TRUE(output.translation().isApprox(initial_pose.translation(), 1e-12));
    EXPECT_NEAR(
        (output.so3().inverse() * initial_pose.so3()).log().norm(),
        0.0,
        1e-12);
}

TEST(ESKFTest, InvalidImuFallbackResetsToLidarPose) {
    LoadTestConfig();
    ESKF eskf;
    eskf.MeasurementUpdateLidar(InitialPose(), kInitialTimestamp);

    const Se3 reset_pose(
        Eigen::AngleAxisd(-0.2, Vec3::UnitZ()).toRotationMatrix(),
        Vec3(4.0, 5.0, 6.0));
    eskf.ResetFromLidar(reset_pose, kInitialTimestamp + 1.0);

    const Se3 output = eskf.GetLocalizationOutput();
    EXPECT_TRUE(output.translation().isApprox(reset_pose.translation(), 1e-12));
    EXPECT_NEAR(
        (output.so3().inverse() * reset_pose.so3()).log().norm(),
        0.0,
        1e-12);
}
