#include "SlamProcess.hpp"
#include "FrontEnd.hpp"
#include "ConfigManager.hpp"
#include "Common.hpp"
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

    void InsertCloud(const std::vector<Vec3>& cloud,
                     const double timestamp) {
        std::lock_guard<std::mutex> lock(cloud_mtx_);
        for (const auto& p : cloud) {
            InsertPoint(p);
        }
        timestamp_ = timestamp;
    }

    void RemoveCloud(const std::vector<Vec3>& cloud){
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
            if(kv.second.count < 15) continue;
            cloud.push_back(kv.second.Centroid());
        }

        return {timestamp_, cloud};
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
    }
    void AddPathPoint(const Vec3& pos) {
        geometry_msgs::msg::PoseStamped pose;

        pose.header.stamp = this->now();
        pose.header.frame_id = "map";

        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.position.z = 0.0;

        pose.pose.orientation.w = 1.0;

        path_.header.stamp = this->now();
        path_.header.frame_id = "map";

        path_.poses.push_back(pose);

        path_pub_->publish(path_);
    }
    void PopFront(){
        path_.poses.erase(path_.poses.begin());
    }

private:

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
    rclcpp::TimerBase::SharedPtr timer_;
};


struct SlamProcess::Impl{
    Impl():voxel_map_(ConfigManager::Get().General_.map_voxel_resolution){
        slam_front_end_.SetKeyFrameCB([this](std::shared_ptr<KeyFrame> key_frame_ptr){

            std::cout<<std::setprecision(15)<<"!!!!!!keyframe :"<<key_frame_ptr->timestamp_<<"\n";   
            Vec3 translation = key_frame_ptr->lio_pose_.translation();
            path_publisher_.AddPathPoint(translation);
            std::vector<Vec3> curr_point_cloud_global;
            submap_queue_.push(key_frame_ptr->saved_frame_path_);
            LoadPLY(key_frame_ptr->saved_frame_path_,curr_point_cloud_global);
            // {
            // std::lock_guard<std::mutex> lock(combine_map_mtx_);
            // auto sensor_pos = key_frame_ptr->lio_pose_.translation();
            // surfel_map_.InsertCloud(curr_point_cloud_global,sensor_pos,key_frame_ptr->timestamp_);    
            // surfel_map_.RemoveWeakSurfels(2.0);
            // surfel_map_.PruneByRange(sensor_pos, 300.0);
            // }
            voxel_map_.InsertCloud(curr_point_cloud_global,key_frame_ptr->timestamp_);
            
            if(submap_queue_.size() > 150){
                auto pop_str = submap_queue_.front();
                std::vector<Vec3> pop_point_cloud_global;
                LoadPLY(pop_str,pop_point_cloud_global);
                voxel_map_.RemoveCloud(pop_point_cloud_global);
                submap_queue_.pop();
                path_publisher_.PopFront();
            }
        });

        slam_front_end_.SetEndFrontEndCB([this](){
            stop_ = true;
        });


        slam_front_end_.Start();
        map_pub_thread = std::thread(&Impl::PublishMap,this);
        
    }

    bool IsFinished() const {
        return stop_;
    }

    void Join(){
        slam_front_end_.Join();
        if(map_pub_thread.joinable()){
            map_pub_thread.join();
        }
    }

    ~Impl(){
        std::cout<<"slam process destructor called\n";
        stop_ = true;
        slam_front_end_.Stop();
        slam_front_end_.Join();
        if(map_pub_thread.joinable()){
            map_pub_thread.join();
        }
    }
    
private:
    void PublishMap(){
        while(!stop_){
            auto updated_map = voxel_map_.GetPointCloud();
            // std::pair<double,std::vector<Vec3>> updated_map = {0.0,{}};
            // {
            // std::lock_guard<std::mutex> lock(combine_map_mtx_);
            // updated_map = surfel_map_.GetStablePoints(4.0);
            // std::cout<<"###########"<<updated_map.second.size()<<"\n";
            // }
            std::cout<<"Publish sub map ##########\n";
            map_publisher_.publish(updated_map.second,updated_map.first);
            // 10 hz publish
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
    // front endcontain dataloading, LIO, submap geerattion
    SlamFrontEnd slam_front_end_;
    VoxelizedMap voxel_map_;
    MapPublisher map_publisher_;
    PathPublisher path_publisher_;
    std::thread map_pub_thread;
    std::thread front_end_thread;
    std::atomic<bool> stop_{false};
    std::queue<std::string> submap_queue_;
    std::mutex combine_map_mtx_;
};

SlamProcess::SlamProcess():process_impl_(std::make_unique<SlamProcess::Impl>()){}

void SlamProcess::Join(){
    process_impl_->Join();
}

bool SlamProcess::IsFinished() const {
    return process_impl_->IsFinished();
}

SlamProcess::~SlamProcess() = default;
