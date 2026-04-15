#pragma once

#include <vector>
#include <Eigen/Dense>
#include "ConfigManager.hpp"
#include "DataType.hpp"

class LoopClosure{
public:
    LoopClosure(){}

    void AddKeyFrame(const std::shared_ptr<KeyFrame>& kf){
        keyframes_list_.push_back(kf);
    }

    std::vector<std::pair<int,Se3>> DetectLoop();

    void SetLastKFId(int last_kf_id){
        last_kf_id_ = last_kf_id;
    }

private:
    
    bool VerifyLoop(const std::shared_ptr<KeyFrame>& kf1, const std::shared_ptr<KeyFrame>& kf2,
                    Se3& relative_pose);


private:
    int last_kf_id_{-1};
    std::vector<std::shared_ptr<KeyFrame>> keyframes_list_;
    double loop_distance_threshold_{ConfigManager::Get().LoopClosure_.loop_closure_search_radius};
    int loop_closure_min_keyframe_gap_{ConfigManager::Get().LoopClosure_.loop_closure_min_keyframe_gap};
    int top_k_best_candidate_{ConfigManager::Get().LoopClosure_.top_k_loop_closure_candidates};
    int skip_count_{ConfigManager::Get().LoopClosure_.skip_count_for_loop_closure_detection};
};