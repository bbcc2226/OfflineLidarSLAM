#include "LoopClosure.hpp"
#include "Common.hpp"
#include "NDT_INC.hpp"

std::vector<std::pair<int,Se3>> LoopClosure::DetectLoop(){
    std::vector<std::pair<int,Se3>> loop_closure_edges;
    if(keyframes_list_.size() < loop_closure_min_keyframe_gap_ || 
        keyframes_list_.back()->key_frame_id_ - last_kf_id_ < skip_count_){
        return loop_closure_edges;
    }else{
        const auto& current_kf = keyframes_list_.back();
        std::vector<std::pair<int,double>> candidate_kf_id_and_distance;
        for(int i = 0; i < static_cast<int>(keyframes_list_.size()) - loop_closure_min_keyframe_gap_; ++i){
            const auto& candidate_kf = keyframes_list_[i];
            double distance = (current_kf->rtk_pos_ - candidate_kf->rtk_pos_).norm();
            if(distance < loop_distance_threshold_){
                candidate_kf_id_and_distance.emplace_back(i,distance);
            }
        }
        // sort the candidate based on distance and only keep top k candidates
        std::sort(candidate_kf_id_and_distance.begin(),candidate_kf_id_and_distance.end(),
                [](const auto& a, const auto& b){ return a.second < b.second;});

        if(candidate_kf_id_and_distance.size() > static_cast<size_t>(top_k_best_candidate_)){
            candidate_kf_id_and_distance.resize(top_k_best_candidate_);
        }
        // further verify the candidate with NDT scan matching and fitness score
        for(const auto& [kf_id, _] : candidate_kf_id_and_distance){
            Se3 relative_pose;
            if(VerifyLoop(current_kf,keyframes_list_[kf_id],relative_pose)){
                loop_closure_edges.emplace_back(keyframes_list_[kf_id]->key_frame_id_, relative_pose);
            }
        }
    }
    // loop closure detection logic
    return loop_closure_edges;
}


bool LoopClosure::VerifyLoop(const std::shared_ptr<KeyFrame>& kf1, const std::shared_ptr<KeyFrame>& kf2,
                            Se3& relative_pose){
    // perform NDT scan matching between the two keyframes and compute the fitness score
    NDT_INC ndt(ConfigManager::Get().LidarOdometry_.ndt_resolution);
    std::shared_ptr<PointCloud> cloud1_ptr = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> cloud2_ptr = std::make_shared<PointCloud>();
    LoadPLY(kf1->saved_frame_path_, cloud1_ptr);
    LoadPLY(kf2->saved_frame_path_, cloud2_ptr);

    InplaceApplyTransform(kf1->lio_pose_, cloud1_ptr);
    InplaceApplyTransform(kf2->lio_pose_, cloud2_ptr);

    ndt.AddCloud(cloud1_ptr);
    Se3 init_guess = kf1->lio_pose_.inverse() * kf2->lio_pose_;
    Se3 optimized_pose = ndt.Align(cloud2_ptr, init_guess);

    double fitness_score = ndt.ComputeFitnessScore(cloud2_ptr, optimized_pose);
    std::cout<<"Loop closure verification between frame "<<kf1->key_frame_id_<<" and frame "<<kf2->key_frame_id_<<" fitness score: "<<fitness_score<<"\n";
    if(fitness_score < ConfigManager::Get().LoopClosure_.loop_closure_fitness_score_threshold){
        relative_pose = optimized_pose;
        return true;
    }else{
        return false;
    }

}
