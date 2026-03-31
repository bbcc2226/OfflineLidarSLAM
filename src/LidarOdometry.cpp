#include "LidarOdometry.hpp"
#include "ConfigManager.hpp"

std::pair<bool,Se3> LidarOdodmetry::AddCloud(std::shared_ptr<PointCloud>& filtered_cloud_ptr, std::shared_ptr<PointCloud>& raw_cloud_ptr, const Se3& predicted_pose, bool use_lo){
    total_cnt_ += 1;
    if(first_frame_){
        first_frame_ = false;
        last_kf_pose_ = Se3();
        ndt_inc_.AddCloud(filtered_cloud_ptr);
        frame_cnt_ += 1;
        key_frame_cnt_ += 1;
        //save the key frame to file
        SaveFrame(raw_cloud_ptr);
        return {true,last_kf_pose_};

    }else{
        Se3 guess = predicted_pose;
        Se3 motion_predicted_pose;
        if(est_pose_buffer_.size() >= 2 ){
            Se3 T1 = est_pose_buffer_[est_pose_buffer_.size() - 1];
            Se3 T2 = est_pose_buffer_[est_pose_buffer_.size() - 2];
            motion_predicted_pose = T1 * (T2.inverse() * T1);
            if(use_lo || frame_cnt_ < 20 ){
                guess = motion_predicted_pose;
            }
        }
        Se3 est_pose = ndt_inc_.Align(filtered_cloud_ptr,guess);
        est_pose_buffer_.push_back(est_pose);
        frame_cnt_ += 1;
        bool key_frame_flag = false;
        if(KeyFrameCheck(est_pose)){
            key_frame_flag = true; 
            frame_cnt_ = 0;
            key_frame_cnt_ += 1;
            last_kf_pose_ = est_pose;
            //std::cout<<est_pose.matrix()<<std::endl;
            std::shared_ptr<PointCloud> curr_filtered_world_cloud = ApplyTransform(est_pose,filtered_cloud_ptr);
            ndt_inc_.AddCloud(curr_filtered_world_cloud);
            //save worl frame cloud (key frame ) to file
            std::shared_ptr<PointCloud> curr_world_cloud = ApplyTransform(est_pose,raw_cloud_ptr);
            SaveFrame(curr_world_cloud);
        }
        return {key_frame_flag,est_pose};
    }
} 


bool LidarOdodmetry::KeyFrameCheck(const Se3& input_pose){

    // large motion jump or frame cnt is over 10
    if(frame_cnt_ > 10){
        return true;
    }
    Se3 delta = last_kf_pose_.inverse() * input_pose;
    
    return delta.translation().norm() > 4.0 ||
            delta.so3().log().norm() > 15.0 / 180.0 * M_PI;
}


void LidarOdodmetry::SaveFrame(const std::shared_ptr<PointCloud>& cloud){
    if(ConfigManager::Get().General_.save_lo_frame){
        const std::string frame_path = GenerateFramePath("./LO_results",key_frame_cnt_);
        SaveCloud(cloud, frame_path,true);
    }
}