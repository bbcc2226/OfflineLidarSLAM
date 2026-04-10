#include "SlamProcess.hpp"
#include "FrontEnd.hpp"
#include "ConfigManager.hpp"
#include "Common.hpp"
#include "Optimizer.hpp"
#include <iostream>
#include <iomanip>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class VoxelizedMap {
public:
    explicit VoxelizedMap(double voxel_size)
        : voxel_size_(voxel_size), timestamp_(0.0) {}

    void InsertPoint(const Vec3& p) {
        VoxelKey key = toKey(p);
        voxel_map_[key].AddPoint(p);
    }

    void InsertCloud(std::vector<Vec3>& cloud, const Se3& sensor_pose,
                     const double timestamp, const int frame_id) {
        for(size_t i = 0; i < cloud.size(); ++i){
            cloud[i] = sensor_pose * cloud[i];
        }
        std::lock_guard<std::mutex> lock(cloud_mtx_);
        for (const auto& p : cloud) {
            InsertPoint(p);
        }
        timestamp_ = timestamp;
        key_frame_poses_[frame_id] = sensor_pose;
    }

    void RemoveCloud(std::vector<Vec3>& cloud, const int frame_id) {
        const auto it = key_frame_poses_.find(frame_id);
        if(it == key_frame_poses_.end()){
            return;
        }
        const Se3& sensor_pose = it->second;
        for(size_t i = 0; i < cloud.size(); ++i){
            cloud[i] = sensor_pose * cloud[i];
        }
        std::lock_guard<std::mutex> lock(cloud_mtx_);
        for (const auto& p : cloud) {
            VoxelKey key = toKey(p);
            auto it = voxel_map_.find(key);
            if (it == voxel_map_.end()) {
                continue;
            }

            it->second.RemovePoint(p);

            if (it->second.Empty()) {
                voxel_map_.erase(it);
            }
        }
    }

    std::pair<double,std::vector<Vec3>> GetPointCloud() const {
        std::lock_guard<std::mutex> lock(cloud_mtx_);
        std::vector<Vec3> cloud;
        cloud.reserve(voxel_map_.size());

        for (const auto& kv : voxel_map_) {
            if(kv.second.count < ConfigManager::Get().General_.num_pts_threshold_for_viz) continue;
            cloud.push_back(kv.second.Centroid());
        }

        return {timestamp_, cloud};
    }
    void RemoveSparseVoxels(int min_neighbors = 4) {
        std::lock_guard<std::mutex> lock(cloud_mtx_);
        std::vector<VoxelKey> voxels_to_remove;

        for (const auto& kv : voxel_map_) {
            const VoxelKey& key = kv.first;

            // skip voxels that don't meet visualization threshold anyway
            if (kv.second.count < ConfigManager::Get().General_.num_pts_threshold_for_viz) continue;

            int neighbor_count = 0;

            // check 26 neighbors
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;

                        VoxelKey neighbor_key{key.x_ + dx, key.y_ + dy, key.z_ + dz};
                        if (voxel_map_.find(neighbor_key) != voxel_map_.end()) {
                            ++neighbor_count;
                        }
                    }
                }
            }

            if (neighbor_count < min_neighbors) {
                voxels_to_remove.push_back(key);
            }
        }

        // remove sparse voxels
        for (const auto& key : voxels_to_remove) {
            voxel_map_.erase(key);
        }

        if (!voxels_to_remove.empty()) {
            std::cout << "[VoxelizedMap] Removed " << voxels_to_remove.size()
                      << " sparse voxels.\n";
        }
    }
private:
    struct VoxelData {
        Vec3 sum = Vec3::Zero();
        int count = 0;

        void AddPoint(const Vec3& p) {
            sum += p;
            ++count;
        }
        void RemovePoint(const Vec3& p) {
            sum -= p;
            --count;
        }
        bool Empty() const {
            return count == 0;
        }
        Eigen::Vector3d Centroid() const {
            return sum / static_cast<double>(count);
        }
    };

    using VoxelMap =
        std::unordered_map<VoxelKey, VoxelData, VoxelKeyHash>;

    VoxelKey toKey(const Vec3& p) const {
        return VoxelKey{
            static_cast<int>(std::floor(p.x() / voxel_size_)),
            static_cast<int>(std::floor(p.y() / voxel_size_)),
            static_cast<int>(std::floor(p.z() / voxel_size_))
        };
    }

    double voxel_size_;
    VoxelMap voxel_map_;
    double timestamp_;
    mutable std::mutex cloud_mtx_;
    std::unordered_map<int,Se3> key_frame_poses_;
};



class MapPublisher : public rclcpp::Node {
public:
    MapPublisher()
        : Node("voxel_map_publisher") {
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/voxel_map", rclcpp::QoS(1).transient_local());
    }

    void publish(
        const std::vector<Vec3>& points,
        const double timestamp)
    {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp.sec = static_cast<int32_t>(timestamp);
        msg.header.stamp.nanosec = static_cast<uint32_t>((timestamp - msg.header.stamp.sec) * 1e9);
        msg.header.frame_id = "map";

        msg.height = 1;
        msg.width = points.size();
        msg.is_dense = true;
        msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        for (const auto& p : points) {
            *iter_x = static_cast<float>(p.x());
            *iter_y = static_cast<float>(p.y());
            *iter_z = static_cast<float>(p.z());
            ++iter_x; ++iter_y; ++iter_z;
        }

        map_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
};


class PathPublisher : public rclcpp::Node {
public:
    PathPublisher() : Node("path_publisher") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory", 10);
        rtk_pub_ = this->create_publisher<nav_msgs::msg::Path>("/rtk_trajectory", 10);
    }
    void AddPathPoint(const Vec3& pos) {
        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = 1.0;

        lio_path_.header.stamp = this->now();
        lio_path_.header.frame_id = "map";

        lio_path_.poses.push_back(pose);

        path_pub_->publish(lio_path_);
    }

    void AddRTKPathPoint(const Vec3& pos, bool valid, double gps_align_yaw) {
        if(!valid){
            rtk_pos_buffer_.push(pos);
            return;
        }else{
            rtk_pos_buffer_.push(pos);
            while(!rtk_pos_buffer_.empty()){
                Vec3 buffered_pos = rtk_pos_buffer_.front();
                rtk_pos_buffer_.pop();
                Eigen::AngleAxisd yaw_rot(gps_align_yaw, Eigen::Vector3d::UnitZ());
                auto corrected_gps_pos = yaw_rot * buffered_pos;

                geometry_msgs::msg::PoseStamped buffered_pose;
                buffered_pose.header.stamp = this->now();
                buffered_pose.header.frame_id = "map";

                buffered_pose.pose.position.x = corrected_gps_pos.x();
                buffered_pose.pose.position.y = corrected_gps_pos.y();
                buffered_pose.pose.position.z = 0.0;

                buffered_pose.pose.orientation.w = 1.0;

                rtk_path_.header.stamp = this->now();
                rtk_path_.header.frame_id = "map";

                rtk_path_.poses.push_back(buffered_pose);
            }

            rtk_pub_->publish(rtk_path_);
        }
    }

    void PopFront(){
        if(!lio_path_.poses.empty()){
            lio_path_.poses.erase(lio_path_.poses.begin());
        }
   
        if(!rtk_path_.poses.empty()){
            rtk_path_.poses.erase(rtk_path_.poses.begin());
        }
    }

private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rtk_pub_;
    std::queue<Vec3> rtk_pos_buffer_;
    nav_msgs::msg::Path lio_path_;
    nav_msgs::msg::Path rtk_path_;
    rclcpp::TimerBase::SharedPtr timer_;
};


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
                for(int i = 0; i < frame_id_collections.size(); ++i){
                    const int processing_frame_id = frame_id_collections[i];
                    const auto lio_pose = key_frame_info_map_[processing_frame_id]->lio_pose_;
                    const auto rtk_pos = key_frame_info_map_[processing_frame_id]->rtk_pos_;
                    std::cout<<"Frame id: "<<processing_frame_id<<" LIO pose: "<< lio_pose.translation().transpose() << " RTK pos: "<<(Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) *rtk_pos).transpose()<<"\n";
                    Eigen::Vector3d t = lio_pose.translation();
                    Eigen::Quaterniond q(lio_pose.rotationMatrix());
                    g2o::SE3Quat T_g2o(q, t);
                    g2o::VertexSE3* v = nullptr;
                    if(i == 0){
                        v =  optimizer_.AddPose(T_g2o, true); // fixed start
                    }else{
                        v = optimizer_.AddPose(T_g2o, false);
                    }
                    active_poses.push_back(v);
                    active_frame_ids.push_back(processing_frame_id);
                }
                // // create yaw node
                //yaw_node = optimizer_.AddYaw(yaw_align_rad_);
                for(size_t i = 0; i < frame_id_collections.size()-1; ++i){
                    const int processing_frame_id = frame_id_collections[i];
                    const int next_processing_frame_id = frame_id_collections[i + 1];
                    const auto curr_lio_pose = key_frame_info_map_[processing_frame_id]->lio_pose_;
                    const auto next_lio_pose = key_frame_info_map_[next_processing_frame_id]->lio_pose_;
                    const auto relative_pose = curr_lio_pose.inverse() * next_lio_pose;
                    const Eigen::Vector3d t = relative_pose.translation();
                    const Eigen::Quaterniond q(relative_pose.rotationMatrix());
                    g2o::SE3Quat relative_g2o(q, t);
                    optimizer_.AddLIOEdge(active_poses[i], active_poses[i+1], relative_g2o);
                }

                for(size_t i = 0; i < frame_id_collections.size(); ++i){
                    const int processing_frame_id = frame_id_collections[i];
                    const auto rtk_pos = key_frame_info_map_[processing_frame_id]->rtk_pos_;
                    //optimizer_.AddGPSEdge(active_poses[i], yaw_node, rtk_pos);
                    const auto rot_rtk_pos = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) * rtk_pos;
                    optimizer_.AddGPSEdge(active_poses[i], rot_rtk_pos);
                }

                // for(size_t i = 0; i < frame_id_collections.size()-1; ++i){
                //     const int processing_frame_id = frame_id_collections[i];
                //     const int next_processing_frame_id = frame_id_collections[i + 1];
                //     const auto rot = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ());
                //     const auto curr_rtk_pos = rot * key_frame_info_map_[processing_frame_id]->rtk_pos_;
                //     const auto next_rtk_pos = rot * key_frame_info_map_[next_processing_frame_id]->rtk_pos_;
                //     const auto relative_rtk_pos = next_rtk_pos - curr_rtk_pos;
                //     //const auto rot_relative_rtk_pos = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) * relative_rtk_pos;
                //     optimizer_.AddGPSRelativeEdge(active_poses[i+1], active_poses[i], relative_rtk_pos);
                // }
            }else{
                const auto pose_info = key_frame_info_map_[current_frame_id]->lio_pose_;
                const auto rtk_pos = key_frame_info_map_[current_frame_id]->rtk_pos_;
                Eigen::Vector3d t = pose_info.translation();
                Eigen::Quaterniond q(pose_info.rotationMatrix());
                g2o::SE3Quat T_g2o(q, t);
                auto v = optimizer_.AddPose(T_g2o, false); // fixed start
                active_poses.push_back(v);
                active_frame_ids.push_back(current_frame_id);

                if (active_poses.size() >= 1) {
                    const int prev_frame_id = active_frame_ids[active_frame_ids.size() - 2];
                    auto curr_pose = key_frame_info_map_[current_frame_id]->lio_pose_;
                    auto pose_prev = key_frame_info_map_[prev_frame_id]->lio_pose_;
                    auto relative_pose = pose_prev.inverse() * curr_pose;
                    Eigen::Vector3d t = relative_pose.translation();
                    Eigen::Quaterniond q(relative_pose.rotationMatrix());
                    g2o::SE3Quat relative_pose_g2o(q, t);
                    auto v_prev = active_poses[active_poses.size() - 2]; 
                    auto v_new = active_poses.back();  
                    optimizer_.AddLIOEdge(v_prev, v_new, relative_pose_g2o);

                    // const auto rot = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ());
                    // const auto curr_rtk_pos = rot *key_frame_info_map_[current_frame_id]->rtk_pos_;
                    // const auto prev_rtk_pos = rot * key_frame_info_map_[prev_frame_id]->rtk_pos_;
                    // std::cout<<"current rtk pos: "<<curr_rtk_pos.transpose() << " prev rtk pos: "<<prev_rtk_pos.transpose()<<"\n";
                    // const auto relative_rtk_pos = curr_rtk_pos - prev_rtk_pos;
                    
                    // //const auto rot_relative_rtk_pos = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) * relative_rtk_pos;
                    // optimizer_.AddGPSRelativeEdge(v_new, v_prev, relative_rtk_pos);
                    // std::cout<<"current pose: "<<pose_info.translation().transpose() << " current rtk pos: "<<curr_rtk_pos.transpose() << " prev rtk pos: "<<prev_rtk_pos.transpose()<<"\n";
                    // std::cout<<"prev pose: "<<pose_prev.translation().transpose() << " prev rtk pos: "<<prev_rtk_pos.transpose()<<"\n";
                    // std::cout<<"Adding relative GPS edge with RTK pos: "<<relative_rtk_pos.transpose()<<"\n";
                    // std::cout<<"Adding LIO edge with relative pose: "<<(curr_pose.translation().transpose() - pose_prev.translation().transpose())<<"\n";
                    // std::cout<<"Current LIO pose: "<<pose_info.translation().transpose()<<"\n";
                }
                const auto rot_rtk_pos = Eigen::AngleAxisd(yaw_align_rad_, Eigen::Vector3d::UnitZ()) * rtk_pos;
                optimizer_.AddGPSEdge(active_poses.back(), rot_rtk_pos);

            }

            auto UpdateVoxelMap = [&](const int key_frame_id, const Se3& optimized_pose){
                //const Se3 se3_pose(optimized_pose.rotation(), optimized_pose.translation());
                const auto key_frame_info = key_frame_info_map_[key_frame_id];
                std::vector<Vec3> curr_point_cloud;
                LoadPLY(key_frame_info->saved_frame_path_,curr_point_cloud);
                voxel_map_.InsertCloud(curr_point_cloud,optimized_pose, key_frame_info->timestamp_,key_frame_id);
                voxel_map_.RemoveSparseVoxels();
                path_publisher_.AddPathPoint(optimized_pose.translation());
                if(key_frame_id > 150 && key_frame_info_map_.find(key_frame_id - 150) != key_frame_info_map_.end()){
                    auto pop_path = key_frame_info_map_[key_frame_id - 150]->saved_frame_path_;
                    std::vector<Vec3> pop_point_cloud_global;
                    LoadPLY(pop_path,pop_point_cloud_global);
                    voxel_map_.RemoveCloud(pop_point_cloud_global, key_frame_info_map_[key_frame_id - 150]->key_frame_id_);
                    path_publisher_.PopFront();
                }
            };

            //Remove old poses outside window (except fixed start) meanwhile update the voxel map
            while (active_poses.size() > ConfigManager::Get().Optimizer_.local_optimization_widnow_size + 1) { // +1 for fixed start
                auto oldest = active_poses[1]; // do not remove fixed start
                auto oldest_frame_id = active_frame_ids[1];
                const Se3 se3_pose(oldest->estimate().rotation(), oldest->estimate().translation());
                UpdateVoxelMap(oldest_frame_id, se3_pose);
                //UpdateVoxelMap(oldest_frame_id, key_frame_info_map_[oldest_frame_id]->lio_pose_); // use the original LIO pose for map update to avoid the map shift during optimization
                
                optimizer_.RemoveVertex(oldest);
                active_poses.erase(active_poses.begin() + 1);
                active_frame_ids.erase(active_frame_ids.begin() + 1);
            }

            // Optimize graph
            optimizer_.Optimize(ConfigManager::Get().Optimizer_.iterations); // 10 iterations per new keyframe

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
    std::thread front_end_thread_;
    std::thread local_graph_optimization_thread_;
    std::atomic<bool> stop_{false};
    std::atomic <bool> rtk_yaw_aligned_{false};
    std::atomic <double> yaw_align_rad_{0.0};
    std::queue<std::shared_ptr<KeyFrame>> key_frame_queue_;
    std::unordered_map<int,std::shared_ptr<KeyFrame>> key_frame_info_map_;
    std::mutex combine_map_mtx_;
    std::mutex key_frame_queue_mtx_;
    std::condition_variable key_frame_queue_cv_;
    GraphOptimizer optimizer_;
};

SlamProcess::SlamProcess():process_impl_(std::make_unique<SlamProcess::Impl>()){}

void SlamProcess::Join(){
    process_impl_->Join();
}

bool SlamProcess::IsFinished() const {
    return process_impl_->IsFinished();
}

SlamProcess::~SlamProcess() = default;
