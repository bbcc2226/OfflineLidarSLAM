#include "NDT_INC.hpp"


void NDT_INC::AddCloud(std::shared_ptr<PointCloud> input_scan_ptr){
    std::unordered_set<VoxelKey,VoxelKeyHash> voxel_set;
    for(const auto& each_pt : input_scan_ptr->pt_list_){
        const Vec3 curr_pt_e = Vec3(each_pt[0],each_pt[1],each_pt[2]);
        const auto curr_key = PointToKey(curr_pt_e);
        auto iter = grids_.find(curr_key);
        if(iter == grids_.end()){
            // new voxel need to be created
            VoxelGaussian_INC new_voxel;
            new_voxel.AddPoint(curr_pt_e);
            data_buffer_.push_front({curr_key,new_voxel});
            grids_.insert({curr_key,data_buffer_.begin()});

            if(data_buffer_.size() > 600){
                // too many points and need to remove the least used pt
                grids_.erase(data_buffer_.back().first);
                data_buffer_.pop_back();
            }
        }else{
            iter->second->second.AddPoint(curr_pt_e);
            data_buffer_.splice(data_buffer_.begin(),data_buffer_,iter->second);
            iter->second = data_buffer_.begin();
        }

        voxel_set.insert(curr_key);
    }
    // parallel update the voxel grip map
    std::for_each(std::execution::par_unseq, voxel_set.begin(), voxel_set.end(),
            [this](const auto& key) {         
                auto it = grids_.find(key);
                if (it != grids_.end()) {
                    UpdateVoxel(it->second->second);
                } 
            });

    if(!initial_){
        initial_ = true;
    }
}

Se3 NDT_INC::Align(std::shared_ptr<PointCloud> input_scan_ptr,const Se3& init_pose, const int max_iter){
    Sophus::SE3d T = init_pose;
    const size_t N = input_scan_ptr->pt_list_.size();

    for (int iter = 0; iter < max_iter; ++iter) {

        // Pre-allocate per-point containers
        std::vector<Eigen::Matrix<double,3,6>> jacobians(N);
        std::vector<Eigen::Vector3d> errors(N);
        std::vector<Eigen::Matrix3d> infos(N);
        std::vector<bool> effect_pts(N);

        // --- 1. Compute residuals & Jacobians in parallel ---
        std::for_each(std::execution::par_unseq, input_scan_ptr->pt_list_.begin(), input_scan_ptr->pt_list_.end(),
            [&](const Point3D& pt){
                size_t idx = &pt - &input_scan_ptr->pt_list_[0];

                Eigen::Vector3d point(pt[0], pt[1], pt[2]);
                Eigen::Vector3d trans_p = T * point;

                const VoxelKey voxel_key = PointToKey(trans_p);
                auto it = grids_.find(voxel_key);
                if (it == grids_.end()) {
                    effect_pts[idx] = false;
                    return;
                }

                const auto& voxel = it->second->second;
                Eigen::Vector3d r = trans_p - voxel.mean_;

                // Optional chi2 threshold
                double res = r.transpose() * voxel.inv_cov_ * r;
                if (std::isnan(res) || res > 9.21) {
                    effect_pts[idx] = false;
                    return;
                }

                // Jacobian
                Eigen::Matrix<double,3,6> J;
                J.block<3,3>(0,0) = -Sophus::SO3d::hat(trans_p);
                J.block<3,3>(0,3) = Eigen::Matrix3d::Identity();

                jacobians[idx] = J;
                errors[idx] = r;
                infos[idx] = voxel.inv_cov_;
                effect_pts[idx] = true;
            }
        );

        // --- 2. Accumulate H and b ---
        Eigen::Matrix<double,6,6> H = Eigen::Matrix<double,6,6>::Zero();
        Eigen::Matrix<double,6,1> b = Eigen::Matrix<double,6,1>::Zero();

        for (size_t i = 0; i < N; ++i) {
            if (!effect_pts[i]) continue;
            const auto& J = jacobians[i];
            const auto& r = errors[i];
            const auto& info = infos[i];

            H += J.transpose() * info * J;
            b += -J.transpose() * info * r;
        }

        // --- 3. Solve for update ---
        Eigen::Matrix<double,6,1> dx = H.ldlt().solve(b);
        //std::cout << "Iter " << iter << ", |dx| = " << dx.norm() << std::endl;
        if (dx.norm() < 0.01) break;

        // Apply Left Update
        Eigen::Matrix<double,6,1> se3_tangent;
        se3_tangent << dx.tail<3>(), dx.head<3>();  // translation, rotation
        T = Sophus::SE3d::exp(se3_tangent) * T;
    }

    return T;
}

void NDT_INC::UpdateVoxel(VoxelGaussian_INC& voxel_data){
    if(!initial_){
        if(voxel_data.pts_.size() > 1){
            ComputeMeanAndVariance(voxel_data,voxel_data.mean_,voxel_data.cov_);
            voxel_data.inv_cov_ = (voxel_data.cov_ + Mat3::Identity() * 1e-3).inverse();
        }else{
            // not enough points
            voxel_data.mean_ = voxel_data.pts_[0];
            voxel_data.inv_cov_ = Mat3::Identity() * 1e2;
        }
        voxel_data.ndt_estimated_ = true;
        voxel_data.pts_.clear();
        return;
    }
    if(!voxel_data.ndt_estimated_ && voxel_data.pts_.size() > 1){
        // this voxel not updated and treated as new voxel
        ComputeMeanAndVariance(voxel_data,voxel_data.mean_,voxel_data.cov_);
        voxel_data.inv_cov_ = (voxel_data.cov_ + Mat3::Identity() * 1e-3).inverse();
        voxel_data.ndt_estimated_ = true;
        voxel_data.pts_.clear();
    }else if(voxel_data.ndt_estimated_ && voxel_data.pts_.size() > 3){
        // dynamicall update the mean and covariance 
        UpdateMeanAndVariance(voxel_data);
        // sanity check the corvariacne
        Eigen::JacobiSVD svd(voxel_data.cov_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Vec3 lambda = svd.singularValues();
        if (lambda[1] < lambda[0] * 1e-3) {
            lambda[1] = lambda[0] * 1e-3;
        }

        if (lambda[2] < lambda[0] * 1e-3) {
            lambda[2] = lambda[0] * 1e-3;
        }

        Mat3 inv_lambda = Vec3(1.0 / lambda[0], 1.0 / lambda[1], 1.0 / lambda[2]).asDiagonal();
        voxel_data.inv_cov_ = svd.matrixV() * inv_lambda * svd.matrixU().transpose();
    }
}

void NDT_INC::ComputeMeanAndVariance(VoxelGaussian_INC& voxel_data, Vec3& mean, Mat3& cov){
    const int N = voxel_data.pts_.size();
    
    for(auto& pt : voxel_data.pts_){
        mean += pt;
    }
    mean /= static_cast<double>(N);

    for (const auto& p : voxel_data.pts_) {
        Eigen::Vector3d diff = p - mean;
        cov += diff * diff.transpose();
    }
    cov /= static_cast<double>(N-1);
}

void NDT_INC::UpdateMeanAndVariance(VoxelGaussian_INC& voxel_data){
    const int M = voxel_data.pts_.size();
    if(M < 3){
        return;
    }
    const int N = voxel_data.num_pts_;
    
    Vec3 mean_B = Vec3::Zero();
    Mat3 cov_B = Mat3::Zero();
    ComputeMeanAndVariance(voxel_data,mean_B,cov_B);

    if (N < 3) {
        voxel_data.mean_ = mean_B;
        voxel_data.cov_  = cov_B;
        voxel_data.num_pts_ = M;
        voxel_data.pts_.clear();
        return;
    }
    const int NM = N + M;
    Vec3 delta = mean_B - voxel_data.mean_;
    Vec3 new_mean = (N * voxel_data.mean_ + M * mean_B) / NM;
    Mat3 scatter =(N - 1) * voxel_data.cov_ + (M - 1) * cov_B
        + (static_cast<double>(N) * M / NM) * (delta * delta.transpose());

    Mat3 new_cov = scatter / static_cast<double>(NM - 1);

    // NDT regularization
    constexpr double eps = 1e-6;
    new_cov += eps * Mat3::Identity();

    voxel_data.mean_ = new_mean;
    voxel_data.cov_ = new_cov;
    voxel_data.num_pts_ = NM;
    voxel_data.pts_.clear();

}

VoxelKey NDT_INC::PointToKey(const Eigen::Vector3d& pt){
    return {
        static_cast<int>(std::floor(pt[0] / resolution_)),
        static_cast<int>(std::floor(pt[1] / resolution_)),
        static_cast<int>(std::floor(pt[2] / resolution_)),             
    };
}