#include "ESKF.hpp"
#include "ConfigManager.hpp"
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
    // Q process noise
    Mat15 Q_{Mat15::Zero()};
    // error state [pos, vel, rot, acc_bias, gyro_bias] --> 15 X 1
    Vec15 dx_{Vec15::Zero()};
    // gravity
    Vec3 g_{0.0, 0.0, -9.81};

    Impl();
    Impl(const Impl & other) = delete;
    Impl & operator=(const Impl & other) = delete;


    void Prediction(const std::shared_ptr<IMUdata>& input_imu);

    void MeasurementUpdate(const Sophus::SE3d& input_pose, const double input_timestamp);

    void UpdateReset();

    void ProjectCov();

    Se3 GetLocalizationOutput() const {
        return Se3(r_,p_);
    }
};


ESKF::Impl::Impl(){
    // inital the process noise in prediction step
    Q_.block<3, 3>(3, 3) = Mat3::Identity() * ConfigManager::Get().ESKF_.process_vel_noise;
    Q_.block<3, 3>(6, 6) = Mat3::Identity() * ConfigManager::Get().ESKF_.process_rot_noise;
    Q_.block<3, 3>(9, 9) = Mat3::Identity() * ConfigManager::Get().ESKF_.process_ba_noise;
    Q_.block<3, 3>(12, 12) = Mat3::Identity() * ConfigManager::Get().ESKF_.process_bg_noise;

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
    
    const double dt = input_imu->timestamp_ - timestamp_;
    Vec3 gravity_;
    gravity_ << 0.0,0.0,9.81;
    // update the new state base on input imu
    // std::cout<<"vel: "<<v_<<"\n";
    // std::cout<<" "<<dt<<" "<<input_imu->acc_<<" "<<input_imu->gyro_<<"\n";
    Vec3 new_p = p_ + v_ * dt + 0.5 * (r_ * (input_imu->acc_ - ba_ ) + g_) * dt * dt;
    Vec3 new_v = v_ + (r_ * (input_imu->acc_ - ba_) + g_) * dt;
    So3 new_r = r_ * So3::exp((input_imu->gyro_ -bg_) * dt);
    p_ = new_p;
    v_ = new_v;
    r_ = new_r;

    Mat15 F = Mat15::Identity();
    F.block<3, 3>(0, 3) = Mat3::Identity() * dt;
    F.block<3, 3>(3, 6) = -r_.matrix() * So3::hat(input_imu->acc_ - ba_) * dt;
    F.block<3, 3>(3, 12) = -r_.matrix() * dt;
    F.block<3, 3>(6, 6) = So3::exp(-(input_imu->gyro_ - bg_) * dt).matrix();
    F.block<3, 3>(6, 9) = -Mat3::Identity() * dt;
    dx_ = F * dx_;
    //update the covarnace matrix
    P_ = F * P_ * F.transpose() + Q_;
    prev_imu_data_ = input_imu;
    timestamp_ = input_imu->timestamp_;
}

void ESKF::Impl::MeasurementUpdate(const Sophus::SE3d& T_meas,const double input_timestamp){
    if(!initialized_){
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

Se3 ESKF::GetLocalizationOutput() const {
    return eskf_impl_->GetLocalizationOutput();
}


ESKF::~ESKF() = default;