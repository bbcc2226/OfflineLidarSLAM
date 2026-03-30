#include "FrontEnd.hpp"
#include "TicToc.hpp"
#include "Config.hpp"
#include "DataLoader.hpp"
#include "ESKF.hpp"
#include "LidarOdometry.hpp"
#include "DataType.hpp"
#include "GeoConverter.hpp"

#include <memory>
#include <queue>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;

struct RTKYawAlignment{
    std::vector<Vec2> rtk_pos_list_;
    std::vector<Vec2> lio_pos_list_;
    bool finished_ {false};
    // rotate from ENU to LIO
    double yaw_rad_ {0.0};
    RTKYawAlignment() = default;
    double NormalizeAngle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
    void PushPos(const Vec2& rtk_pos, const Vec2& lio_pos){
        if(!finished_){
            rtk_pos_list_.push_back(rtk_pos);
            lio_pos_list_.push_back(lio_pos);
            TryEstimate();
        }
    }

    void TryEstimate(){
        if (lio_pos_list_.size() != rtk_pos_list_.size()) {
            throw std::runtime_error("Invalid input size");
        }
        const double min_motion = 0.2;
        if(lio_pos_list_.size() == 10){
            double sum_sin = 0.0;
            double sum_cos = 0.0;
            int valid_cnt = 0;

            for (size_t i = 0; i + 1 < lio_pos_list_.size(); ++i) {
                Vec2 dp_lio = lio_pos_list_[i + 1] - lio_pos_list_[i];
                Vec2 dp_gps = rtk_pos_list_[i + 1] - rtk_pos_list_[i];

                double norm_lio = dp_lio.norm();
                double norm_gps = dp_gps.norm();

                if (norm_lio < min_motion || norm_gps < min_motion)
                    continue;

                double yaw_lio = std::atan2(dp_lio.y(), dp_lio.x());
                double yaw_gps = std::atan2(dp_gps.y(), dp_gps.x());

                double yaw_offset = NormalizeAngle(yaw_gps - yaw_lio);

                sum_sin += std::sin(yaw_offset);
                sum_cos += std::cos(yaw_offset);
                valid_cnt++;
            }

            if(valid_cnt > 5){
                yaw_rad_ = -std::atan2(sum_sin,sum_cos);
                finished_  = true;
            }
        }
    }

    bool GetValid() const {
        return finished_;
    }

};

struct SlamFrontEnd::Impl{

public:
    Impl();

    void Start();

    void Join();

    void Stop();

    void SetKeyFrameCB(KFCallback kf_cb){
        kf_cb_ = kf_cb;
    }

    void SetEndFrontEndCb(EndFrontEndCallback fe_cb){
        fe_cb_ = fe_cb;
    }

    ~Impl();
private:
    LidarOdodmetry lo_;
    SensorDataPlayer data_loader_;
    ESKF state_estimator_;
    GeoConverter geo_converter_;
    RTKYawAlignment rtk_align_yaw_;
    KFCallback kf_cb_;
    EndFrontEndCallback fe_cb_;
    
    int cloud_id_ {0};
    double latest_imu_timestamp_{0.0};
    double latest_gps_timestamp_{0.0};

    std::mutex buffer_mtx_;
    std::mutex lidar_process_mtx_;
    std::thread monitor_thread_;
    std::thread process_thread_;
    std::atomic<bool> stop_ {false};
    std::atomic<bool> finish_data_loading_ {false};
    std::atomic<int> cloud_pq_size_{0};
    std::condition_variable lidar_buffer_cv_;
    std::vector<std::thread> thread_pool_lidar_;

    std::deque<std::pair<int,std::shared_ptr<PointCloud>>> lidar_cloud_buffer_;
    std::deque<std::shared_ptr<IMUdata>> imu_buffer_;
    std::deque<std::shared_ptr<GPSdata>> gps_buffer_;
    std::vector<std::shared_ptr<KeyFrame>> uncorrected_keyframe_;

    struct CloudLess{
        bool operator()(const PointcloudComb& a, const PointcloudComb& b) const {
            return a.frame_id_ > b.frame_id_;
        }
    };

    std::priority_queue<PointcloudComb,std::vector<PointcloudComb>,CloudLess> cloud_pq_; 

    void PushLidarData(std::shared_ptr<PointCloud> lidar_ptr_data);

    void PushImuData(std::shared_ptr<IMUdata> imu_data);

    void PushGPSData(std::shared_ptr<GPSdata> gps_data);

    void MonitorStatus();

    void ProcessSensorData();

    void PointCloudDownSample();

    std::string SaveLIOFrame(Se3& global_pose,const std::shared_ptr<PointCloud>& cloud, const int frame_id);
};


SlamFrontEnd::Impl::Impl():lo_(Config::LidarOdometry::ndt_resolution),data_loader_(Config::DataLoader::ros_bag_path){
    // bind the callback function
    data_loader_.SetLidarCallback(
        [this](std::shared_ptr<PointCloud> lidar_ptr){
            this->PushLidarData(lidar_ptr);
        }
    );
    data_loader_.SetIMUCallback(
        [this](std::shared_ptr<IMUdata> imu_ptr){
            this->PushImuData(imu_ptr);
        }
    );
    data_loader_.SetGPSCallback(
        [this](std::shared_ptr<GPSdata> gps_ptr){
            this->PushGPSData(gps_ptr);
        }
    );
    data_loader_.SetFinishCallback(
        [this](){
            this->finish_data_loading_ = true;
            
        }
    );

}


void SlamFrontEnd::Impl::Start(){
    // start data loader
    data_loader_.Start();
    // create monitor thread
    monitor_thread_ = std::thread(&SlamFrontEnd::Impl::MonitorStatus, this);
    process_thread_ = std::thread(&SlamFrontEnd::Impl::ProcessSensorData, this);

    for(int i = 0; i< 3; i++){
        thread_pool_lidar_.push_back(std::thread(&SlamFrontEnd::Impl::PointCloudDownSample,this));
    }

    //Join();
}

void SlamFrontEnd::Impl::Stop(){
    stop_ = true;
    data_loader_.Stop();  
    lidar_buffer_cv_.notify_all();
    //Join();
    if(fe_cb_){
        // tell the backend the front end already finish
        std::cout<<"Front end process finish\n";
        fe_cb_();
    }
}

void SlamFrontEnd::Impl::Join() {
    if (monitor_thread_.joinable()) monitor_thread_.join();
    //std::cout<<"join monitor thread\n";
    if (process_thread_.joinable()) process_thread_.join();
    //std::cout<<"join process thread\n";
    for(auto& t : thread_pool_lidar_){
        if(t.joinable()) t.join();
    }
    //std::cout<<"join worker thread\n";

}

SlamFrontEnd::Impl::~Impl(){
    std::cout<<"front end process destructor called\n";
    Stop();
    Join();
}

void SlamFrontEnd::Impl::PushLidarData(std::shared_ptr<PointCloud> lidar_ptr_data){
    //std::cout<<"receive new LIDAR data \n";
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    cloud_id_ += 1;
    lidar_cloud_buffer_.push_back({cloud_id_, lidar_ptr_data});
    lidar_buffer_cv_.notify_one();
}

void SlamFrontEnd::Impl::PushImuData(std::shared_ptr<IMUdata> imu_data){
    //std::cout<<"receive new IMU data \n";
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    imu_buffer_.push_back(imu_data);
    latest_imu_timestamp_ = std::max(latest_imu_timestamp_,imu_data->timestamp_);
}

void SlamFrontEnd::Impl::PushGPSData(std::shared_ptr<GPSdata> gps_data){
    //std::cout<<"receive new GPS data \n";
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    gps_buffer_.push_back(gps_data);
    latest_gps_timestamp_ = std::max(latest_gps_timestamp_,gps_data->timestamp_);
}

void SlamFrontEnd::Impl::MonitorStatus(){
    while(!stop_){
        int lidar_buffer_size = 0;
        int imu_buffer_size = 0;
        int gps_buffer_size = 0;

        {
        std::lock_guard<std::mutex> lock(buffer_mtx_);
        lidar_buffer_size = lidar_cloud_buffer_.size();
        imu_buffer_size = imu_buffer_.size();
        gps_buffer_size = gps_buffer_.size();
        }

        if(!finish_data_loading_.load()){
            if(data_loader_.HoldStatus()){
                if(cloud_pq_size_ < Config::FrontEnd::lidar_buffer_lower_capacity ){
                    std::cout<<"Resume the data loader!" <<std::endl;
                    data_loader_.Resume();
                
                }
            }else{
                if(cloud_pq_size_ > Config::FrontEnd::lidar_buffer_upper_capacity){
                    std::cout<<"Pause the data loader!" <<std::endl;
                    data_loader_.Pause();
                }
            }
        }

        {
            std::lock_guard<std::mutex>lock(lidar_process_mtx_);
            std::cout<<"domwsample pq size"<<cloud_pq_.size()<<" "<<cloud_pq_size_<<"\n"; 
            if(cloud_pq_.size() == 0 && finish_data_loading_ && lidar_buffer_size == 0){
                std::cout<<"set the front flag stop to True because tasks are finished !\n";
                stop_ = true;
            }
        }
        if(stop_){
           Stop();
        }

        std::cout<<"Lidar buffer size :" << lidar_buffer_size
            <<" Imu buffer size : " << imu_buffer_size
            <<" gps buffer size : " << gps_buffer_size <<"\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void SlamFrontEnd::Impl::ProcessSensorData(){
    int expected_lidar_id = 1;
    while(!stop_){
        PointcloudComb curr_comb;
        {
            std::lock_guard<std::mutex> lock(lidar_process_mtx_);
            if(!cloud_pq_.empty()){
                curr_comb = cloud_pq_.top();
                if(curr_comb.frame_id_ == expected_lidar_id && 
                    curr_comb.timestamp_ <= latest_imu_timestamp_&&
                    curr_comb.timestamp_ <= latest_gps_timestamp_){
                    cloud_pq_.pop();
                    expected_lidar_id += 1;
                    cloud_pq_size_--;
                }else{
                    curr_comb.frame_id_ = 0;
                    curr_comb.timestamp_ = 0.0;
                }
            }
        } // end of lidar process mutex

        // start to process lidar point cloud
        std::vector<std::shared_ptr<IMUdata>> imu_wait_for_processing;
        std::vector<std::shared_ptr<GPSdata>> gps_wait_for_processing;
        
        {
            std::lock_guard<std::mutex> lock(buffer_mtx_);

            while(!imu_buffer_.empty() && imu_buffer_.front()->timestamp_ <= curr_comb.timestamp_){
                imu_wait_for_processing.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();

            }
            while(!gps_buffer_.empty() && gps_buffer_.front()->timestamp_ <= curr_comb.timestamp_){
                gps_wait_for_processing.push_back(gps_buffer_.front());
                gps_buffer_.pop_front();
            }

        }

        if(curr_comb.frame_id_ > 0 && gps_wait_for_processing.size() > 0){
            // has a valid lidar scan 
            Se3 pred_state;
            // check if first Lidar frame been recived and if the 'global' frame get set
            if(lo_.ValidPose()){
                // make the prediction with IMU and ESKF
                for(auto& curr_imu : imu_wait_for_processing){
                    state_estimator_.Prediction(curr_imu);
                }
                pred_state = state_estimator_.GetLocalizationOutput();
                std::cout<<"prediction: "<<pred_state.matrix() <<"\n";
            }
            // query the synced gps for late backend RTK optimization
            auto matched_gps = gps_wait_for_processing.back();
            // curr gps coordiante under ENU
            Vec3 gps_coordinate = geo_converter_.Geo2ENU(matched_gps->pos_);
            std::cout<<matched_gps->timestamp_ - curr_comb.timestamp_<<" "<<gps_coordinate<<"\n";
            // perform the LO to estimate the pse measurement from Lidar scan
            TicToc timer;
            timer.tic();
            std::pair<bool,Se3> est_pose = lo_.AddCloud(curr_comb.filtered_cloud_,curr_comb.raw_cloud_,pred_state,true);
            timer.toc("NDT processing time is: ");
            // measurement update with Lo results and ESKF
            state_estimator_.MeasurementUpdateLidar(est_pose.second,curr_comb.timestamp_);
            std::cout<<"Estimation : "<<est_pose.second.matrix() <<"\n";
            // check if current lidar scan is the key frame from LO processing
            const bool is_keyframe = est_pose.first;
            if(is_keyframe){
                Se3 curr_est_pose = state_estimator_.GetLocalizationOutput();
                std::string keyframe_path = SaveLIOFrame(curr_est_pose,curr_comb.raw_cloud_,curr_comb.frame_id_);
                if(kf_cb_){
                    std::shared_ptr<KeyFrame> curr_key_frame_ptr = std::make_shared<KeyFrame>();
                    curr_key_frame_ptr->timestamp_ = curr_comb.timestamp_;
                    curr_key_frame_ptr->saved_frame_path_ = keyframe_path;
                    curr_key_frame_ptr->valid_rtk_ = true;
                    curr_key_frame_ptr->lio_pose_ = curr_est_pose;
                    curr_key_frame_ptr->rtk_pos_ = gps_coordinate;
                    kf_cb_(curr_key_frame_ptr);
                }
            }
            // for the RTK ENu frame yaw alignment with 'global' frame
            rtk_align_yaw_.PushPos(gps_coordinate.head<2>(),est_pose.second.translation().head<2>());
            if(rtk_align_yaw_.GetValid()){
                std::cout<<"rtk yaw alignment: "<<rtk_align_yaw_.yaw_rad_ / M_PI * 180.0 <<"\n";
                Eigen::AngleAxisd yaw_rot(rtk_align_yaw_.yaw_rad_, Eigen::Vector3d::UnitZ());
                auto corrected_gps_pos = yaw_rot * gps_coordinate;

                std::cout<< corrected_gps_pos <<"\n";
            }
            // else{
            //     // correction angle is not ready and need to push to invalid key frame pool and correct later 
            //     uncorrected_keyframe_.push_back
            // }

            // std::cout<<std::setprecision(15)<<curr_comb.frame_id_<<" "<<curr_comb.timestamp_<<" "<<imu_wait_for_processing.size()
            //     <<" "<<gps_wait_for_processing.size()<<" "<<latest_imu_timestamp_<<"\n";
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void SlamFrontEnd::Impl::PointCloudDownSample(){
    while(true){
        std::shared_ptr<PointCloud> curr_cloud_ptr = nullptr;
        int curr_cloud_id = 0;
        {
        std::unique_lock<std::mutex> lock(buffer_mtx_);
        lidar_buffer_cv_.wait(lock,[&](){
            return stop_ || !lidar_cloud_buffer_.empty();
        });
        
        if(stop_ && lidar_cloud_buffer_.empty()){
            break;
        }
        curr_cloud_id = lidar_cloud_buffer_.front().first;
        curr_cloud_ptr = lidar_cloud_buffer_.front().second;
        lidar_cloud_buffer_.pop_front();
        }

        if(curr_cloud_ptr){
            std::shared_ptr<PointCloud> filtered_cloud = ApplyDownSampleFilter(curr_cloud_ptr);
            PointcloudComb comb;
            comb.frame_id_ = curr_cloud_id;
            comb.timestamp_ = curr_cloud_ptr->timestamp_;
            comb.filtered_cloud_ = filtered_cloud;
            comb.raw_cloud_ = curr_cloud_ptr;
            {
            std::lock_guard<std::mutex> lock(lidar_process_mtx_);
            cloud_pq_.push(comb);
            }
            cloud_pq_size_++;
        }else{
            std::cout<<"here\n";
        }
        
    }
}

std::string SlamFrontEnd::Impl::SaveLIOFrame(Se3& global_pose,const std::shared_ptr<PointCloud>& cloud, const int frame_id){
    if(Config::General::save_lio_frame){
        std::shared_ptr<PointCloud> converted_cloud;
        if(Config::General::filter_saved_cloud){
            auto filter_cloud = ApplyRangeFilter(cloud);
            converted_cloud = ApplyTransform(global_pose,filter_cloud);
        }else{
            converted_cloud = ApplyTransform(global_pose,cloud);
        }
        fs::path dir = Config::FrontEnd::lio_dir_path;
        if (!fs::exists(dir)) {
            if (fs::create_directory(dir)) {
                std::cout << "Directory created\n";
            } else {
                std::cout << "Failed to create directory\n";
            }
        }

        const std::string frame_path = GenerateFramePath(Config::FrontEnd::lio_dir_path,frame_id);
        SaveCloud(converted_cloud, frame_path);
        return frame_path;
    }else{
        return "";
    }
}

SlamFrontEnd::SlamFrontEnd():frontend_impl_(std::make_unique<SlamFrontEnd::Impl>()){

}

SlamFrontEnd::~SlamFrontEnd() = default;

void SlamFrontEnd::Start(){
    frontend_impl_->Start();
}

void SlamFrontEnd::Stop(){
    frontend_impl_->Stop();
}

void SlamFrontEnd::Join(){
    frontend_impl_->Join();
}

void SlamFrontEnd::SetKeyFrameCB(KFCallback kf_cb){
    frontend_impl_->SetKeyFrameCB(kf_cb);
}

void SlamFrontEnd::SetEndFrontEndCB(EndFrontEndCallback fe_cb){
    frontend_impl_->SetEndFrontEndCb(fe_cb);
}