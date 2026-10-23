#pragma once

#include <vector>
#include <unordered_map>
#include <limits>
#include <string>
#include <Eigen/Dense>
#include "ConfigManager.hpp"
#include "DataType.hpp"

struct LoopClosureVerificationMetrics {
    double ndt_score{std::numeric_limits<double>::quiet_NaN()};
    double gicp_score{std::numeric_limits<double>::quiet_NaN()};
    double correction_translation{std::numeric_limits<double>::quiet_NaN()};
    double correction_rotation_deg{std::numeric_limits<double>::quiet_NaN()};
    bool gicp_converged{false};
    std::string reason{"not_evaluated"};
};

class LoopClosure{
public:
    LoopClosure();

    void AddKeyFrame(const std::shared_ptr<KeyFrame>& kf){
        double cumulative_distance = 0.0;
        if (!keyframes_list_.empty()) {
            cumulative_distance = cumulative_travel_distance_.back() +
                (kf->lio_pose_.translation() -
                 keyframes_list_.back()->lio_pose_.translation()).norm();
        }
        cumulative_travel_distance_.push_back(cumulative_distance);
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
                    Se3& relative_pose, LoopClosureVerificationMetrics& metrics);

    bool VerifyLoopSubmap(const std::shared_ptr<KeyFrame>& curr_frame, const std::vector<std::shared_ptr<KeyFrame>>& history_frames,
                    Se3& relative_pose, LoopClosureVerificationMetrics& metrics);

    void RecordLoopCheck(int current_frame_id,
                         int history_frame_id,
                         double spatial_distance,
                         double travel_distance,
                         bool accepted,
                         const LoopClosureVerificationMetrics& metrics) const;
private:
    int last_kf_id_{-1};
    std::vector<std::shared_ptr<KeyFrame>> keyframes_list_;
    std::vector<double> cumulative_travel_distance_;
    double loop_distance_threshold_{ConfigManager::Get().LoopClosure_.loop_closure_search_radius};
    int loop_closure_min_keyframe_gap_{ConfigManager::Get().LoopClosure_.loop_closure_min_keyframe_gap};
    double loop_closure_min_travel_distance_{ConfigManager::Get().LoopClosure_.loop_closure_min_travel_distance};
    int top_k_best_candidate_{ConfigManager::Get().LoopClosure_.top_k_loop_closure_candidates};
    int skip_count_{ConfigManager::Get().LoopClosure_.skip_count_for_loop_closure_detection};
    std::string loop_closure_log_path_{ConfigManager::Get().LoopClosure_.loop_closure_log_path};
};