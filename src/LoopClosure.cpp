#include "LoopClosure.hpp"
#include "Common.hpp"
#include "NDT_INC.hpp"
#include <fstream>

std::pair<int,Se3> LoopClosure::DetectLoop(){
    std::pair<int,Se3> loop_closure_edge = {-1, Se3()};
    if(keyframes_list_.size() < loop_closure_min_keyframe_gap_ || 
        keyframes_list_.back()->key_frame_id_ - last_kf_id_ < skip_count_){
        return loop_closure_edge;
    }else{
        const auto& current_kf = keyframes_list_.back();
        int cloest_id = -1;
        double cloest_distance = std::numeric_limits<double>::max();
        for(int i = 0; i < static_cast<int>(keyframes_list_.size()) - loop_closure_min_keyframe_gap_; ++i){
            const auto& candidate_kf = keyframes_list_[i];
            double distance = (current_kf->rtk_pos_ - candidate_kf->rtk_pos_).norm();
            const int keyframe_id_gap = current_kf->key_frame_id_ - candidate_kf->key_frame_id_;
            if(distance < loop_distance_threshold_ && keyframe_id_gap > loop_closure_min_keyframe_gap_){
                if(distance < cloest_distance){
                    cloest_distance = distance;
                    cloest_id = i;
                }
            }
        }
        if(cloest_id != -1){
            std::vector<std::shared_ptr<KeyFrame>> candidate_history_frames;
            const int map_half_window = 5;
            for(int i = std::max(0, cloest_id - map_half_window); i <= std::min(static_cast<int>(keyframes_list_.size()) - 1, cloest_id + map_half_window); ++i){
                candidate_history_frames.push_back(keyframes_list_[i]);
            }
            bool loop_accepted = false;
            Se3 submap_world_pose;
            loop_accepted = VerifyLoopSubmap(current_kf, candidate_history_frames, submap_world_pose);
            if(loop_accepted){
                const auto& best_kf = keyframes_list_[cloest_id];
                Se3 relative_vs_best = best_kf->optimized_pose_.second.inverse() * submap_world_pose;
                loop_closure_edge = std::make_pair(best_kf->key_frame_id_, relative_vs_best);
                std::cout<<"Loop closure accepted: frame "<<current_kf->key_frame_id_
                         <<" -> "<<best_kf->key_frame_id_<<"\n";
            }
            // Se3 submap_world_pose;
            // if(VerifyLoopSubmap(current_kf, candidate_history_frames, submap_world_pose)){
            //     // Convert absolute world pose to relative edge vs. best (closest) history candidate
            //     const auto& best_kf = candidate_history_frames[0];
            //     Se3 relative_vs_best = best_kf->optimized_pose_.second.inverse() * submap_world_pose;
            //     loop_closure_edge = std::make_pair(best_kf->key_frame_id_, relative_vs_best);
            //     std::cout<<"Submap loop closure accepted: frame "<<current_kf->key_frame_id_
            //              <<" -> "<<best_kf->key_frame_id_<<"\n";
            // }
        }
    }
    // loop closure detection logic
    return loop_closure_edge;
}


bool LoopClosure::VerifyLoop(const std::shared_ptr<KeyFrame>& kf1, const std::shared_ptr<KeyFrame>& kf2,
                            Se3& relative_pose){
    // perform NDT scan matching between the two keyframes and compute the fitness score
    NDT_INC ndt(8.0);
    VoxelFilter filter(0.7);
    std::shared_ptr<PointCloud> cloud1_ptr = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> cloud2_ptr = std::make_shared<PointCloud>();
    LoadPLY(kf1->saved_frame_path_, cloud1_ptr);
    LoadPLY(kf2->saved_frame_path_, cloud2_ptr);
    cloud1_ptr = filter.Downsample(cloud1_ptr);
    cloud2_ptr = filter.Downsample(cloud2_ptr);

    ndt.AddCloud(cloud2_ptr);
    Se3 init_guess = kf2->optimized_pose_.second.inverse() * kf1->optimized_pose_.second;
    Se3 optimized_pose = ndt.Align(cloud1_ptr, init_guess, 60);

    double fitness_score = ndt.ComputeFitnessScore(cloud1_ptr, optimized_pose);
    std::cout<<"Loop closure verification between frame "<<kf1->key_frame_id_<<" and frame "<<kf2->key_frame_id_<<" fitness score: "<<fitness_score<<"\n";
    if(fitness_score < ConfigManager::Get().LoopClosure_.loop_closure_fitness_score_threshold){
        relative_pose = optimized_pose;
        return true;
    }else{
        return false;
    }

}

bool LoopClosure::VerifyLoopSubmap(const std::shared_ptr<KeyFrame>& curr_frame, const std::vector<std::shared_ptr<KeyFrame>>& history_frames,
                            Se3& relative_pose){

    NDT_INC ndt(10.0);  // large capacity: submap may span many frames
    VoxelFilter filter(0.7);
    std::shared_ptr<PointCloud> submap_ptr = std::make_shared<PointCloud>();
    for(const auto& kf : history_frames){
        std::shared_ptr<PointCloud> cloud_ptr = std::make_shared<PointCloud>();
        LoadPLY(kf->saved_frame_path_, cloud_ptr);
        cloud_ptr = filter.Downsample(cloud_ptr);
        InplaceApplyTransform(kf->optimized_pose_.second, cloud_ptr);
        submap_ptr->pt_list_.insert(submap_ptr->pt_list_.end(),
                                    cloud_ptr->pt_list_.begin(), cloud_ptr->pt_list_.end());
        ndt.AddCloud(cloud_ptr);
    }
    std::shared_ptr<PointCloud> curr_cloud_ptr = std::make_shared<PointCloud>();
    LoadPLY(curr_frame->saved_frame_path_, curr_cloud_ptr);
    curr_cloud_ptr = filter.Downsample(curr_cloud_ptr);
    Se3 init_guess = curr_frame->optimized_pose_.second;
    Se3 optimized_pose = ndt.Align(curr_cloud_ptr, init_guess, 60);

    double fitness_score = ndt.ComputeFitnessScore(curr_cloud_ptr, optimized_pose);
    std::cout<<"Loop closure submap verification for frame "<<curr_frame->key_frame_id_<<" fitness score: "<<fitness_score<<"\n";

    // Save submap, raw current scan, initial guess, and aligned scan for visualization/replay
    if(ConfigManager::Get().General_.save_loop_closure_debug_info){
        const int id = curr_frame->key_frame_id_;
        std::string submap_path   = "/tmp/loop_submap_kf"       + std::to_string(id) + ".ply";
        std::string raw_path      = "/tmp/loop_curr_raw_kf"     + std::to_string(id) + ".ply";
        std::string aligned_path  = "/tmp/loop_curr_aligned_kf" + std::to_string(id) + ".ply";
        std::string init_path     = "/tmp/loop_init_guess_kf"   + std::to_string(id) + ".txt";
        SaveCloud(submap_ptr, submap_path);
        SaveCloud(curr_cloud_ptr, raw_path);
        std::shared_ptr<PointCloud> aligned_curr = ApplyTransform(optimized_pose, curr_cloud_ptr);
        SaveCloud(aligned_curr, aligned_path);
        // Save initial guess as: tx ty tz qw qx qy qz
        Eigen::Quaterniond q = init_guess.unit_quaternion();
        Eigen::Vector3d    t = init_guess.translation();
        std::ofstream ofs(init_path);
        ofs << t.x() << " " << t.y() << " " << t.z() << " "
            << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
        std::cout<<"Saved submap to "<<submap_path<<", raw scan to "<<raw_path
                 <<", init guess to "<<init_path<<", aligned scan to "<<aligned_path<<"\n";
    }
    if(fitness_score < ConfigManager::Get().LoopClosure_.loop_closure_fitness_score_threshold){
        relative_pose = optimized_pose;
        return true;
    }else{
        return false;
    }
}
