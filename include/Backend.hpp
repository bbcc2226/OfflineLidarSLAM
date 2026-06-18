#pragma once

#include "DataType.hpp"
#include "LoopClosure.hpp"
#include "Optimizer.hpp"
#include "Tools/MapPublisher.hpp"
#include "Tools/PathPublisher.hpp"
#include "VoxelizedMap.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>

class Backend {
public:
    Backend();
    ~Backend();

    Backend(const Backend&) = delete;
    Backend& operator=(const Backend&) = delete;

    void PushKeyFrame(const std::shared_ptr<KeyFrame>& key_frame_ptr);
    void RequestStop();
    void Join();
    bool IsFinished() const;

private:
    void PublishMap();
    void LocalGraphOptimization();

    VoxelizedMap voxel_map_;
    MapPublisher map_publisher_;
    PathPublisher path_publisher_;

    std::thread map_pub_thread_;
    std::thread local_graph_optimization_thread_;
    std::mutex combine_map_mtx_;
    std::mutex key_frame_queue_mtx_;
    std::condition_variable key_frame_queue_cv_;

    std::atomic<bool> stop_{false};
    std::atomic<bool> local_graph_done_{false};
    std::atomic<int> pending_keyframes_{0};
    std::atomic<bool> rtk_yaw_aligned_{false};
    std::atomic<double> yaw_align_rad_{0.0};

    std::queue<std::shared_ptr<KeyFrame>> key_frame_queue_;
    std::unordered_map<int, std::shared_ptr<KeyFrame>> key_frame_info_map_;
    std::string keyframe_json_output_path_;

    GraphOptimizer local_optimizer_;
    GraphOptimizer global_optimizer_;
    LoopClosure loop_closure_detector_;
    std::unordered_map<int, g2o::VertexSE3*> loop_closure_anchor_map_;
};