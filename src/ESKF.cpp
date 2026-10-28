#include "ESKF.hpp"
#include "ConfigManager.hpp"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

struct ESKF::Impl{
    bool initialized_{false};
    double timestamp_ {0.0};
    std::shared_ptr<IMUdata> prev_imu_data_;

    // kalman filter state;
    // position under ENU frame
    Vec3 p_{Vec3::Zero()};
    // velocity under ENU frame
    Vec3 v_{Vec3::Zero()};
    // rotation vector 
    So3 r_;
    // acc bias
    Vec3 ba_{Vec3::Zero()};
    // gyro bias
    Vec3 bg_{Vec3::Zero()};

    // P cov matrix
    Mat15 P_{Mat15::Identity()};
    Mat3 acc_noise_cov_{Mat3::Zero()};
    Mat3 gyro_noise_cov_{Mat3::Zero()};
    Mat3 acc_bias_noise_cov_{Mat3::Zero()};
    Mat3 gyro_bias_noise_cov_{Mat3::Zero()};
    // error state [pos, vel, rot, acc_bias, gyro_bias] --> 15 X 1
    Vec15 dx_{Vec15::Zero()};
    Vec3 g_{Vec3::Zero()};
    Vec3 initial_specific_force_body_{Vec3::Zero()};
    double max_imu_dt_{0.05};
    size_t skipped_large_dt_count_{0};

    Impl();
    Impl(const Impl & other) = delete;
    Impl & operator=(const Impl & other) = delete;


    void Prediction(const std::shared_ptr<IMUdata>& input_imu);

    void MeasurementUpdate(const Sophus::SE3d& input_pose, const double input_timestamp);

    void ResetFromLidar(const Sophus::SE3d& input_pose, double input_timestamp);

    void UpdateReset();

    void ProjectCov();

    Se3 GetLocalizationOutput() const {
        return Se3(r_,p_);
    }
};


ESKF::Impl::Impl(){
    const auto& cfg = ConfigManager::Get().ESKF_;
    acc_noise_cov_ = Mat3::Identity() * cfg.process_vel_noise;
    gyro_noise_cov_ = Mat3::Identity() * cfg.process_rot_noise;
    acc_bias_noise_cov_ = Mat3::Identity() * cfg.process_ba_noise;
    gyro_bias_noise_cov_ = Mat3::Identity() * cfg.process_bg_noise;
    ba_ << cfg.initial_acc_bias_x,
           cfg.initial_acc_bias_y,
           cfg.initial_acc_bias_z;
    bg_ << cfg.initial_gyro_bias_x,
           cfg.initial_gyro_bias_y,
           cfg.initial_gyro_bias_z;
    max_imu_dt_ = cfg.max_imu_dt;

    const double roll = cfg.initial_roll_deg * M_PI / 180.0;
    const double pitch = cfg.initial_pitch_deg * M_PI / 180.0;
    const Eigen::Matrix3d body_to_level =
        (Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
         Eigen::AngleAxisd(roll, Vec3::UnitX())).toRotationMatrix();
    initial_specific_force_body_ =
        body_to_level.transpose() *
        Vec3(0.0, 0.0, cfg.gravity_magnitude);
}

void ESKF::Impl::UpdateReset(){
    //update the nominal state after the measurement update step
    p_ += dx_.block<3, 1>(0, 0);
    v_ += dx_.block<3, 1>(3, 0);
    r_ = r_ * So3::exp(dx_.block<3, 1>(6, 0));
    ba_ += dx_.block<3, 1>(9, 0);
    bg_ += dx_.block<3, 1>(12, 0);
    ProjectCov();
    dx_ = Vec15::Zero();
}

void ESKF::Impl::ProjectCov(){
    // correct the covariance matrix after the measurement update
    Mat15 J = Mat15::Identity();
    J.block<3, 3>(6, 6) = Mat3::Identity() - 0.5 * So3::hat(dx_.block<3, 1>(6, 0));
    P_ = J * P_ * J.transpose();
}


void ESKF::Impl::Prediction(const std::shared_ptr<IMUdata>& input_imu){
    if (!initialized_) {
        return;
    }

    const double dt = input_imu->timestamp_ - timestamp_;
    if (dt <= 0.0) {
        std::cerr << "Skipping non-monotonic IMU sample: dt=" << dt << "\n";
        return;
    }
    if (dt > max_imu_dt_) {
        ++skipped_large_dt_count_;
        if (skipped_large_dt_count_ == 1 ||
            skipped_large_dt_count_ % 100 == 0) {
            std::cerr << "Skipping IMU propagation across gap: dt=" << dt
                      << " s, skipped gaps=" << skipped_large_dt_count_ << "\n";
        }
        prev_imu_data_ = input_imu;
        timestamp_ = input_imu->timestamp_;
        return;
    }

    const Mat3 rotation = r_.matrix();
    const Vec3 corrected_acc = input_imu->acc_ - ba_;
    const Vec3 corrected_gyro = input_imu->gyro_ - bg_;
    const Vec3 world_acc = rotation * corrected_acc + g_;
    const double dt_squared = dt * dt;

    // Linearize about the state before applying the nominal update.
    Mat15 F = Mat15::Identity();
    F.block<3, 3>(0, 3) = Mat3::Identity() * dt;
    F.block<3, 3>(0, 6) =
        -0.5 * rotation * So3::hat(corrected_acc) * dt_squared;
    F.block<3, 3>(0, 9) = -0.5 * rotation * dt_squared;
    F.block<3, 3>(3, 6) =
        -rotation * So3::hat(corrected_acc) * dt;
    F.block<3, 3>(3, 9) = -rotation * dt;
    F.block<3, 3>(6, 6) =
        So3::exp(-corrected_gyro * dt).matrix();
    F.block<3, 3>(6, 12) = -Mat3::Identity() * dt;

    Mat15 Q_discrete = Mat15::Zero();
    const Mat3 world_acc_noise = rotation * acc_noise_cov_ * rotation.transpose();
    Q_discrete.block<3, 3>(0, 0) =
        world_acc_noise * (dt * dt_squared / 3.0);
    Q_discrete.block<3, 3>(0, 3) =
        world_acc_noise * (dt_squared / 2.0);
    Q_discrete.block<3, 3>(3, 0) =
        Q_discrete.block<3, 3>(0, 3).transpose();
    Q_discrete.block<3, 3>(3, 3) = world_acc_noise * dt;
    Q_discrete.block<3, 3>(6, 6) = gyro_noise_cov_ * dt;
    Q_discrete.block<3, 3>(9, 9) = acc_bias_noise_cov_ * dt;
    Q_discrete.block<3, 3>(12, 12) = gyro_bias_noise_cov_ * dt;

    dx_ = F * dx_;
    P_ = F * P_ * F.transpose() + Q_discrete;
    P_ = 0.5 * (P_ + P_.transpose());

    p_ += v_ * dt + 0.5 * world_acc * dt_squared;
    v_ += world_acc * dt;
    r_ = r_ * So3::exp(corrected_gyro * dt);

    prev_imu_data_ = input_imu;
    timestamp_ = input_imu->timestamp_;
}

void ESKF::Impl::MeasurementUpdate(const Sophus::SE3d& T_meas,const double input_timestamp){
    if(!initialized_){
        p_ = T_meas.translation();
        v_.setZero();
        r_ = T_meas.so3();
        g_ = -(r_ * initial_specific_force_body_);
        dx_.setZero();
        initialized_ = true;
        timestamp_ = input_timestamp;
        return;
    }
    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    H.block<3, 3>(0, 0) = Mat3::Identity();
    H.block<3, 3>(3, 6) = Mat3::Identity();
    R.block<3, 3>(0, 0) = Mat3::Identity() * ConfigManager::Get().ESKF_.measurement_pos_noise;
    R.block<3, 3>(3, 3) = Mat3::Identity() * ConfigManager::Get().ESKF_.measurement_rot_noise;

    Sophus::SE3d T_hat(r_, p_);

    // Pose error
    Sophus::SE3d T_err = T_hat.inverse() * T_meas;
    Eigen::Matrix<double,6,1> se3_err = T_err.log();

    // std::cout<<"measuremnt R :" << T_meas.so3().matrix().eulerAngles(2, 1, 0)<<"\n";
    // std::cout<<"prediction R :" << r_.matrix().eulerAngles(2, 1, 0)<<"\n";
    // std::cout<<"measuremnt T :" << T_meas.translation()<<"\n";
    // std::cout<<"prediction T :" << p_<<"\n";
    // Log map
    Eigen::Matrix<double,6,1> innov = Eigen::Matrix<double,6,1>::Zero();
    innov.template head<3>() = (T_meas.translation() - p_);          
    innov.template tail<3>() = (r_.inverse() * T_meas.so3()).log();  

    auto k = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();

    dx_ = k * innov;

    // use Joseph formula to update the covariance matrix
    P_ = (Mat15::Identity() - k * H) * P_ * (Mat15::Identity() - k * H).transpose() + k * R * k.transpose();

    // update the reset
    UpdateReset();
    timestamp_ = input_timestamp;
}

void ESKF::Impl::ResetFromLidar(
    const Sophus::SE3d& input_pose,
    double input_timestamp) {
    p_ = input_pose.translation();
    v_.setZero();
    r_ = input_pose.so3();
    dx_.setZero();
    P_ = Mat15::Identity();
    if (!initialized_) {
        g_ = -(r_ * initial_specific_force_body_);
        initialized_ = true;
    }
    timestamp_ = input_timestamp;
}


ESKF::ESKF(): eskf_impl_(std::make_unique<Impl>()){

}

void ESKF::Prediction(std::shared_ptr<IMUdata> input_imu)
{
  eskf_impl_->Prediction(std::move(input_imu));
}

void ESKF::MeasurementUpdateLidar(const Se3& input_pose, const double input_timestamp)
{
  eskf_impl_->MeasurementUpdate(input_pose, input_timestamp);
}

void ESKF::ResetFromLidar(
    const Se3& input_pose,
    double input_timestamp) {
  eskf_impl_->ResetFromLidar(input_pose, input_timestamp);
}

Se3 ESKF::GetLocalizationOutput() const {
    return eskf_impl_->GetLocalizationOutput();
}


ESKF::~ESKF() = default;
