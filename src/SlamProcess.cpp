#include "SlamProcess.hpp"
#include "FrontEnd.hpp"
#include "ConfigManager.hpp"
#include "Common.hpp"
#include "Optimizer.hpp"
#include "LoopClosure.hpp"
#include "Tools/KeyFrameJsonIO.hpp"
#include "Tools/MapPublisher.hpp"
#include "Tools/PathPublisher.hpp"
#include "VoxelizedMap.hpp"
#include <iostream>
#include <iomanip>
#include <queue>


struct SlamProcess::Impl{
    Impl():voxel_map_(ConfigManager::Get().General_.map_voxel_resolution){
        
        slam_front_end_.SetKeyFrameCB([this](std::shared_ptr<KeyFrame> key_frame_ptr){

            //std::cout<<std::setprecision(15)<<"keyframe :"<<key_frame_ptr->timestamp_<<"\n";   
            {
            // push the latest key frame info to queue for graph optimization
            std::lock_guard<std::mutex> lock(key_frame_queue_mtx_);
            key_frame_queue_.push(key_frame_ptr);
            key_frame_queue_cv_.notify_one();
            }
            // update the inital aligned yaw for rtk
            if(key_frame_ptr->valid_rtk_ && !rtk_yaw_aligned_){
                yaw_align_rad_ = key_frame_ptr->rtk_align_yaw_;
                rtk_yaw_aligned_ = true;
            }
        });

        slam_front_end_.SetEndFrontEndCB([this](){
            stop_ = true;
            key_frame_queue_cv_.notify_all();
        });


        slam_front_end_.Start();
        map_pub_thread_ = std::thread(&Impl::PublishMap,this);
        local_graph_optimization_thread_ = std::thread(&Impl::LocalGraphOptimization,this);
        
    }

    bool IsFinished() const {
        return stop_;
    }

    void Join(){
        slam_front_end_.Join();
        if(map_pub_thread_.joinable()){
            map_pub_thread_.join();
        }
        if(local_graph_optimization_thread_.joinable()){
            local_graph_optimization_thread_.join();
        }
    }

    ~Impl(){
        std::cout<<"slam process destructor called\n";
        stop_ = true;
        slam_front_end_.Stop();
        slam_front_end_.Join();
        if(map_pub_thread_.joinable()){
            map_pub_thread_.join();
        }
        if(local_graph_optimization_thread_.joinable()){
            local_graph_optimization_thread_.join();
        }
    }
    
private:
    void PublishMap(){
        while(!stop_){
            auto updated_map = voxel_map_.GetPointCloud();
            // std::cout<<"Publish sub map\n";
            map_publisher_.publish(updated_map.second,updated_map.first);
            // 1 hz publish
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    void LocalGraphOptimization(){
        std::deque<g2o::VertexSE3*> active_poses;
        std::deque<int> active_frame_ids;
        std::vector<int> frame_id_collections;
        std::unordered_set<int> processed_map_frame_id;
        Se3 corrected_pose( Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero());
        std::unordered_map<int, Se3> post_loop_closure_poses; // globally-optimized poses saved after loop closure
        // VertexYaw* yaw_node = nullptr;
         
        while(true){
            // check if there is new key frame added and do local graph optimization
            std::unique_lock<std::mutex> lock(key_frame_queue_mtx_);
            key_frame_queue_cv_.wait(lock, [this] { return !key_frame_queue_.empty() || stop_; });
            if(stop_) break;
            auto key_frame = key_frame_queue_.front();
            key_frame_queue_.pop();
            lock.unlock();
            // do local graph optimization
            const int current_frame_id = key_frame->key_frame_id_;
            frame_id_collections.push_back(current_frame_id);
            key_frame_info_map_[current_frame_id] = key_frame;

            if(!rtk_yaw_aligned_){
                // wait for the rtk yaw to be aligned before doing optimization, just keep adding the edge info to list and continue
                continue;
            }
            if (active_poses.empty()) {
                for(size_t i = 0; i < frame_id_collections.size(); ++i){
                    const int processing_frame_id = frame_id_collections[i];
                    Se3 lio_pose;
                    {
                        auto it_saved = post_loop_closure_poses.find(processing_frame_id);
                        lio_pose = (it_saved != post_loop_closure_poses.end())
                                   ? it_saved->second
                                   : corrected_pose * key_frame_info_map_[processing_frame_id]->lio_pose_;
                    }
                    const auto rtk_pos = key_frame_info_map_[processing_frame_id]->rtk_pos_;
                    std::cout<<"Frame id: "<<processing_frame_id<<" LIO pose: "<< lio_pose.translation().transpose() << " RTK pos: "<<(Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) *rtk_pos).transpose()<<"\n";
                    Eigen::Vector3d t = lio_pose.translation();
                    Eigen::Quaterniond q(lio_pose.rotationMatrix());
                    g2o::SE3Quat T_g2o(q, t);
                    g2o::VertexSE3* v = nullptr;
                    if(i == 0){
                        v =  local_optimizer_.AddPose(T_g2o, true); // fixed start
                        if(loop_closure_anchor_map_.find(processing_frame_id) == loop_closure_anchor_map_.end()){
                            loop_closure_anchor_map_[processing_frame_id] = global_optimizer_.AddPose(T_g2o, true);
                        }

                    }else{
                        v = local_optimizer_.AddPose(T_g2o, false);
                        if(loop_closure_anchor_map_.find(processing_frame_id) == loop_closure_anchor_map_.end()){
                            loop_closure_anchor_map_[processing_frame_id] = global_optimizer_.AddPose(T_g2o, false);
                        }
                    }
                    active_poses.push_back(v);
                    active_frame_ids.push_back(processing_frame_id);

                }
                // // create yaw node
                //yaw_node = optimizer_.AddYaw(yaw_align_rad_);
                for(size_t i = 0; i < frame_id_collections.size()-1; ++i){
                    const int processing_frame_id = frame_id_collections[i];
                    const int next_processing_frame_id = frame_id_collections[i + 1];
                    const auto curr_lio_pose = corrected_pose * key_frame_info_map_[processing_frame_id]->lio_pose_;
                    const auto next_lio_pose = corrected_pose * key_frame_info_map_[next_processing_frame_id]->lio_pose_;
                    const auto relative_pose = curr_lio_pose.inverse() * next_lio_pose;
                    const Eigen::Vector3d t = relative_pose.translation();
                    const Eigen::Quaterniond q(relative_pose.rotationMatrix());
                    g2o::SE3Quat relative_g2o(q, t);
                    local_optimizer_.AddLIOEdge(active_poses[i], active_poses[i+1], relative_g2o);
                    global_optimizer_.AddLIOEdge(loop_closure_anchor_map_[processing_frame_id], loop_closure_anchor_map_[next_processing_frame_id], relative_g2o);
                }

                for(size_t i = 0; i < frame_id_collections.size(); ++i){
                    const int processing_frame_id = frame_id_collections[i];
                    const auto rtk_pos = key_frame_info_map_[processing_frame_id]->rtk_pos_;
                    const auto lio_pose = corrected_pose * key_frame_info_map_[processing_frame_id]->lio_pose_;
                     std::cout<<"Frame id: "<<processing_frame_id<<" LIO pose: "<< lio_pose.translation().transpose() << " RTK pos: "<<(Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) *rtk_pos).transpose()<<"\n";
                    //optimizer_.AddGPSEdge(active_poses[i], yaw_node, rtk_pos);
                    const auto rot_rtk_pos = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) * rtk_pos;
                    local_optimizer_.AddGPSEdge(active_poses[i], rot_rtk_pos);
                    global_optimizer_.AddGPSEdge(loop_closure_anchor_map_[processing_frame_id], rot_rtk_pos);
                    // Se3 gps_with_heading(lio_pose.rotationMatrix(), rot_rtk_pos);
                    //optimizer_.AddGPSWithHeadingEdge(active_poses[i], gps_with_heading);
                }
            }else{
                const auto pose_info = corrected_pose * key_frame_info_map_[current_frame_id]->lio_pose_;
                const auto rtk_pos = key_frame_info_map_[current_frame_id]->rtk_pos_;
                Eigen::Vector3d t = pose_info.translation();
                Eigen::Quaterniond q(pose_info.rotationMatrix());
                g2o::SE3Quat T_g2o(q, t);
                auto v = local_optimizer_.AddPose(T_g2o, false); // fixed start
                auto v_global = global_optimizer_.AddPose(T_g2o, false);
                loop_closure_anchor_map_[current_frame_id] = v_global;
                active_poses.push_back(v);
                active_frame_ids.push_back(current_frame_id);

                if (active_poses.size() >= 1) {
                    const int prev_frame_id = active_frame_ids[active_frame_ids.size() - 2];
                    auto curr_pose = corrected_pose * key_frame_info_map_[current_frame_id]->lio_pose_;
                    auto pose_prev = corrected_pose * key_frame_info_map_[prev_frame_id]->lio_pose_;
                    auto relative_pose = pose_prev.inverse() * curr_pose;
                    Eigen::Vector3d t = relative_pose.translation();
                    Eigen::Quaterniond q(relative_pose.rotationMatrix());
                    g2o::SE3Quat relative_pose_g2o(q, t);
                    auto v_prev = active_poses[active_poses.size() - 2]; 
                    auto v_new = active_poses.back();  
                    local_optimizer_.AddLIOEdge(v_prev, v_new, relative_pose_g2o);
                    global_optimizer_.AddLIOEdge(loop_closure_anchor_map_[prev_frame_id], loop_closure_anchor_map_[current_frame_id], relative_pose_g2o);
                }
                const auto rot_rtk_pos = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) * rtk_pos;
                local_optimizer_.AddGPSEdge(active_poses.back(), rot_rtk_pos);
                global_optimizer_.AddGPSEdge(loop_closure_anchor_map_[current_frame_id], rot_rtk_pos);
                //Se3 gps_with_heading(pose_info.rotationMatrix(), rot_rtk_pos);
                //optimizer_.AddGPSWithHeadingEdge(active_poses.back(), gps_with_heading);
            }

            auto UpdateVoxelMap = [&](const int key_frame_id, const Se3& optimized_pose){
                if(processed_map_frame_id.find(key_frame_id) != processed_map_frame_id.end()){
                    return; // already updated
                }
                //const Se3 se3_pose(optimized_pose.rotation(), optimized_pose.translation());
                const auto key_frame_info = key_frame_info_map_[key_frame_id];
                std::vector<Vec3> curr_point_cloud;
                LoadPLY(key_frame_info->saved_frame_path_,curr_point_cloud);
                voxel_map_.InsertCloud(curr_point_cloud,optimized_pose, key_frame_info->timestamp_,key_frame_id);
                voxel_map_.RemoveSparseVoxels();
                path_publisher_.AddPathPoint(optimized_pose.translation());
                path_publisher_.AddRTKPathPoint(key_frame_info->rtk_pos_,
                                               key_frame_info->valid_rtk_,
                                               yaw_align_rad_);
                // if(key_frame_id > 250 && key_frame_info_map_.find(key_frame_id - 250) != key_frame_info_map_.end()){
                //     auto pop_path = key_frame_info_map_[key_frame_id - 250]->saved_frame_path_;
                //     std::vector<Vec3> pop_point_cloud_global;
                //     LoadPLY(pop_path,pop_point_cloud_global);
                //     voxel_map_.RemoveCloud(pop_point_cloud_global, key_frame_info_map_[key_frame_id - 250]->key_frame_id_);
                //     path_publisher_.PopFront();
                // }
            };

            auto RefreshMapAndPath = [&](){
                voxel_map_.Clear();
                processed_map_frame_id.clear();

                // Collect and sort by frame_id so path is in temporal order
                std::vector<std::pair<int, Se3>> sorted_optimized;
                std::unordered_map<int, Se3> id_to_pose;
                for (auto& kv : loop_closure_anchor_map_) {
                    const auto& est = kv.second->estimate();
                    Se3 pose(est.rotation(), est.translation());
                    sorted_optimized.emplace_back(kv.first, pose);
                    id_to_pose[kv.first] = pose;
                }
                std::sort(sorted_optimized.begin(), sorted_optimized.end(),
                          [](const auto& a, const auto& b){ return a.first < b.first; });

                // Update stored optimized_pose_ in loop closure detector so future
                // NDT verifications use the globally-corrected world-frame poses.
                loop_closure_detector_.UpdateKeyFramePoses(id_to_pose);

                // Rebuild voxel map
                for (auto& [frame_id, pose] : sorted_optimized) {
                    processed_map_frame_id.insert(frame_id);
                    const auto key_frame_info = key_frame_info_map_[frame_id];
                    std::vector<Vec3> curr_point_cloud;
                    LoadPLY(key_frame_info->saved_frame_path_, curr_point_cloud);
                    voxel_map_.InsertCloud(curr_point_cloud, pose, key_frame_info->timestamp_, frame_id);
                }
                voxel_map_.RemoveSparseVoxels();

                // Rebuild path from sorted globally-optimized positions
                std::vector<Vec3> path_positions;
                path_positions.reserve(sorted_optimized.size());
                std::vector<Vec3> rtk_positions;
                rtk_positions.reserve(sorted_optimized.size());
                for (const auto& [frame_id, pose] : sorted_optimized) {
                    path_positions.push_back(pose.translation());
                    const auto key_frame_info = key_frame_info_map_[frame_id];
                    if (key_frame_info->valid_rtk_) {
                        rtk_positions.push_back(Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) * key_frame_info->rtk_pos_);
                    }
                }
                path_publisher_.RefreshPath(path_positions);
                path_publisher_.RefreshRTKPath(rtk_positions);
            };

            //Remove old poses outside window (except fixed start) meanwhile update the voxel map
            std::pair<int,Se3> loop_closure_edge;
            int processed_frame_id = -1;
            while (active_poses.size() > static_cast<size_t>(ConfigManager::Get().Optimizer_.local_optimization_widnow_size + 1)) { // +1 for fixed start
                auto oldest = active_poses[0]; // do not remove fixed start
                processed_frame_id = active_frame_ids[0];
                const Se3 se3_pose(oldest->estimate().rotation(), oldest->estimate().translation());
                tools::RecordKeyFrameToJson(processed_frame_id, se3_pose, key_frame_info_map_, keyframe_json_output_path_);
                if(ConfigManager::Get().General_.using_LIO_only){
                    UpdateVoxelMap(processed_frame_id, key_frame_info_map_[processed_frame_id]->lio_pose_); // use the original LIO pose for map update to avoid the map shift during optimization
                } else {
                    UpdateVoxelMap(processed_frame_id, se3_pose);
                }
                auto optimized_key_frame_info = key_frame_info_map_[processed_frame_id];
                optimized_key_frame_info->optimized_pose_ = {true, se3_pose};
                loop_closure_detector_.AddKeyFrame(optimized_key_frame_info);
                local_optimizer_.RemoveVertex(oldest);
                active_poses.erase(active_poses.begin());
                active_frame_ids.erase(active_frame_ids.begin());
                auto curr_oldest = active_poses.empty() ? nullptr : active_poses[0];
                if(curr_oldest){
                    curr_oldest->setFixed(true); // fix the oldest vertex after optimization to avoid the shift of the whole graph
                }
            }

            // global Optimize graph
            // Apply any loop closure edges detected this iteration
            loop_closure_edge = loop_closure_detector_.DetectLoop();

            if(loop_closure_edge.first != -1 ){
                const int hist_kf_id = loop_closure_edge.first;
                const Se3& rel_pose  = loop_closure_edge.second;

                auto hist_kf_info = key_frame_info_map_[hist_kf_id];
                auto hist_kf_global_vertex = loop_closure_anchor_map_[hist_kf_id];
                auto current_kf_global_vertex = loop_closure_anchor_map_[processed_frame_id]; // processed_frame_id = keyframes_list_.back()
                g2o::SE3Quat rel_pose_g2o(Eigen::Quaterniond(rel_pose.rotationMatrix()), rel_pose.translation());
                global_optimizer_.AddLoopClosureEdge(hist_kf_global_vertex, current_kf_global_vertex, rel_pose_g2o);
                global_optimizer_.Optimize(ConfigManager::Get().Optimizer_.global_iterations); // optimize the global graph first to get better loop closure constraint
                // world-frame correction: T_world_correction = T_opt * T_LIO_curr^{-1}
                // applied LEFT: corrected_pose * T_LIO_i = T_opt * T_LIO_curr^{-1} * T_LIO_i
                corrected_pose = Se3(current_kf_global_vertex->estimate().rotation(), current_kf_global_vertex->estimate().translation()) * key_frame_info_map_[processed_frame_id]->lio_pose_.inverse();
                std::cout<<"Loop closure detected between frame "<<processed_frame_id<<" and frame "<<hist_kf_id<<"\n";
                std::cout<<"Corrected pose after loop closure: "<<corrected_pose.translation().transpose()<<"\n";
                RefreshMapAndPath();
                // after global optimization, update the local graph with the optimized poses in the global graph to    get better local optimization result
                frame_id_collections.clear();
                for(size_t i = 0; i < active_frame_ids.size(); ++i){
                    frame_id_collections.push_back(active_frame_ids[i]);
                }
                local_optimizer_.Clear();
                active_poses.clear();
                active_frame_ids.clear();
                loop_closure_detector_.SetLastKFId(current_frame_id); // set the last key frame id for loop closure detection to avoid detecting loop closure with the key frames that have been optimized and removed from the local graph
            }

            if(active_poses.size() > 1){
                local_optimizer_.Optimize(ConfigManager::Get().Optimizer_.local_iterations); // 10 iterations per new keyframe
            }

            for(size_t i = 0; i < active_poses.size(); ++i){
                std::cout <<active_frame_ids[i]<< " Pose " << i << ": " << active_poses[i]->estimate().translation().transpose() << "\n";
            }
            // Print results
            //std::cout << "Estimated yaw: " << yaw_node->estimate() * 180.0 / M_PI << " deg\n";

        }
    }
    // front endcontain dataloading, LIO, submap geerattion
    SlamFrontEnd slam_front_end_;
    VoxelizedMap voxel_map_;

    MapPublisher map_publisher_;
    PathPublisher path_publisher_;

    std::thread map_pub_thread_;
    std::thread local_graph_optimization_thread_;
    std::mutex combine_map_mtx_;
    std::mutex key_frame_queue_mtx_;
    std::condition_variable key_frame_queue_cv_;

    std::atomic<bool> stop_{false};
    std::atomic <bool> rtk_yaw_aligned_{false};
    std::atomic <double> yaw_align_rad_{0.0};

    std::queue<std::shared_ptr<KeyFrame>> key_frame_queue_;
    std::unordered_map<int,std::shared_ptr<KeyFrame>> key_frame_info_map_;
    std::string keyframe_json_output_path_;

    GraphOptimizer local_optimizer_;
    GraphOptimizer global_optimizer_;
    LoopClosure loop_closure_detector_;
    // Fixed anchor vertices for loop closure targets (keyed by keyframe id)
    std::unordered_map<int, g2o::VertexSE3*> loop_closure_anchor_map_;
};

SlamProcess::SlamProcess():process_impl_(std::make_unique<SlamProcess::Impl>()){}

void SlamProcess::Join(){
    process_impl_->Join();
}

bool SlamProcess::IsFinished() const {
    return process_impl_->IsFinished();
}

SlamProcess::~SlamProcess() = default;
