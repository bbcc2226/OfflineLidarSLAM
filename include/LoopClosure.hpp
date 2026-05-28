#pragma once

#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include "ConfigManager.hpp"
#include "DataType.hpp"

class LoopClosure{
public:
    LoopClosure(){}

    void AddKeyFrame(const std::shared_ptr<KeyFrame>& kf){
        keyframes_list_.push_back(kf);
    }

    std::pair<int,Se3> DetectLoop();

    void SetLastKFId(int last_kf_id){
        last_kf_id_ = last_kf_id;
    }

    // Update the stored optimized_pose_ for all keyframes after global optimization,
    // so future loop closure NDT uses the correct world-frame initial guess.
    void UpdateKeyFramePoses(const std::unordered_map<int, Se3>& id_to_pose) {
        for (auto& kf : keyframes_list_) {
            auto it = id_to_pose.find(kf->key_frame_id_);
            if (it != id_to_pose.end()) {
                kf->optimized_pose_ = {true, it->second};
            }
        }
    }

private:
    
    bool VerifyLoop(const std::shared_ptr<KeyFrame>& kf1, const std::shared_ptr<KeyFrame>& kf2,
                    Se3& relative_pose);

    bool VerifyLoopSubmap(const std::shared_ptr<KeyFrame>& curr_frame, const std::vector<std::shared_ptr<KeyFrame>>& history_frames,
                    Se3& relative_pose);
private:
    int last_kf_id_{-1};
    std::vector<std::shared_ptr<KeyFrame>> keyframes_list_;
    double loop_distance_threshold_{ConfigManager::Get().LoopClosure_.loop_closure_search_radius};
    int loop_closure_min_keyframe_gap_{ConfigManager::Get().LoopClosure_.loop_closure_min_keyframe_gap};
    int top_k_best_candidate_{ConfigManager::Get().LoopClosure_.top_k_loop_closure_candidates};
    int skip_count_{ConfigManager::Get().LoopClosure_.skip_count_for_loop_closure_detection};
};