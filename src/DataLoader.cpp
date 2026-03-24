#include "DataLoader.hpp"

    SensorDataPlayer::SensorDataPlayer(const std::string& input_data_path): data_path_(input_data_path){
        // Setup ROS bag reader
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = data_path_;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        reader_.open(storage_options, converter_options);

        imu_serializer_ = std::make_unique<rclcpp::Serialization<sensor_msgs::msg::Imu>>();
        gps_serializer_ = std::make_unique<rclcpp::Serialization<sensor_msgs::msg::NavSatFix>>();
        lidar_serializer_ = std::make_unique<rclcpp::Serialization<sensor_msgs::msg::PointCloud2>>();
        
    }

    void SensorDataPlayer::Start(){
        std::cout<<"start data loader \n";
        loading_thread_ = std::thread(&SensorDataPlayer::LoadingThread,this);   
    }

    void SensorDataPlayer::Stop(){
        stop_ = true;
        task_cv_.notify_all();
        if(loading_thread_.joinable()){
            loading_thread_.join();
        }
        if(f_cb_){
            f_cb_();
        }
    }

    void SensorDataPlayer::LoadingThread(){
        pthread_setname_np(pthread_self(), "data_load");

        while(stop_ == false){
            std::unique_lock<std::mutex> lock(loading_mutex_);
            task_cv_.wait(lock,[&]{
                return stop_ || hold_ == false;
            });

            if(stop_ || !reader_.has_next()){
                std::cout<<"end the loading process \n";
                // kill the loading process
                stop_ = true;
                hold_ = false;
                lock.unlock();
                task_cv_.notify_all();
                if(f_cb_){
                    f_cb_();
                }   
                break;
            }
            // loading the next bag message
            auto bag_msg = reader_.read_next();
            auto sensor_msg = rclcpp::SerializedMessage(*bag_msg->serialized_data);
            
            if(bag_msg->topic_name == imu_topic_){
                //std::cout<<"sending imu message \n";
                ProcessIMUMsg(sensor_msg);
            }else if(bag_msg->topic_name == gps_topic_){
                //std::cout<<"sending gps message \n";
                ProcessGPSMsg(sensor_msg);
            }else if(bag_msg->topic_name == lidar_topic_){
                //std::cout<<"sending lidar message \n";
                ProcessLidarMsg(sensor_msg);
            }
        }
    }

    void SensorDataPlayer::ProcessIMUMsg(rclcpp::SerializedMessage& msg){
        sensor_msgs::msg::Imu imu_msg;
        imu_serializer_->deserialize_message(&msg, &imu_msg);
        std::shared_ptr<IMUdata> imu_data = std::make_shared<IMUdata>();
        imu_data->timestamp_ = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;
        imu_data->acc_ << imu_msg.linear_acceleration.x,
                            imu_msg.linear_acceleration.y,
                            imu_msg.linear_acceleration.z;
        imu_data->gyro_ << imu_msg.angular_velocity.x,
                            imu_msg.angular_velocity.y,
                            imu_msg.angular_velocity.z;
        // std::cout<<std::setprecision(15)<<"imu: "<<imu_data->timestamp_<<std::endl;
        if(imu_cb_){
            imu_cb_(imu_data);
        }
        
    }

    void SensorDataPlayer::ProcessLidarMsg(rclcpp::SerializedMessage& msg){
        sensor_msgs::msg::PointCloud2 cloud_msg;
        lidar_serializer_->deserialize_message(&msg, &cloud_msg);
        std::shared_ptr<PointCloud> lidar_cloud = std::make_shared<PointCloud>();
        lidar_cloud->timestamp_ = cloud_msg.header.stamp.sec + cloud_msg.header.stamp.nanosec * 1e-9;
        // std::cout<<std::setprecision(15)<<"lidar: "<<lidar_cloud->timestamp_<<std::endl;

        // Check if intensity exists
        bool has_intensity = false;
        for (const auto &field : cloud_msg.fields) {
            if (field.name == "intensity") {
                has_intensity = true;
                break;
            }
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_i(cloud_msg, "intensity"); // if exists

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
            //remove the ground
            if(*iter_z <= -1.0) continue;
            lidar_cloud->pt_list_.emplace_back(Point3D{*iter_x, *iter_y, *iter_z});
        }
        if(lidar_cb_){
            lidar_cb_(lidar_cloud);
        }
    }

    void SensorDataPlayer::ProcessGPSMsg(rclcpp::SerializedMessage& msg){
        sensor_msgs::msg::NavSatFix gps_msg;
        gps_serializer_->deserialize_message(&msg, &gps_msg);
        std::shared_ptr<GPSdata> gps_data = std::make_shared<GPSdata>();
        gps_data->timestamp_ = gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec * 1e-9;
        gps_data->pos_<< gps_msg.latitude, gps_msg.longitude, gps_msg.altitude;
        if(gps_cb_){
            gps_cb_(gps_data);
        }
    }






