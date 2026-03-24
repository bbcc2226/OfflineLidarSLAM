#ifndef __DATALOADER__
#define __DATALOADER__


#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rclcpp/serialization.hpp>
#include "rclcpp/time.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "DataType.hpp"

#include <pthread.h>
#include <thread>
#include <queue>
#include <mutex>
#include <functional>
#include <condition_variable>
#include <atomic>
#include <string>
#include <iostream>


class SensorDataPlayer{
public:
    using IMUCallback = std::function<void(std::shared_ptr<IMUdata>)>;
    using LidarCallback = std::function<void(std::shared_ptr<PointCloud>)>;
    using GPSCallback = std::function<void(std::shared_ptr<GPSdata>)>;
    using FinshCallback = std::function<void()>;
    explicit SensorDataPlayer(const std::string& input_data_path);

    // start the data loading process
    void Start();

    void Stop();

    // Pause the data loading process
    void Pause(){
        hold_ = true;
        task_cv_.notify_all();
    }

    // resume the data loading process
    void Resume(){
        hold_ = false;
        task_cv_.notify_all();
    }

    bool HoldStatus() const {
        return hold_.load();
    }

    void SetLidarCallback(LidarCallback lidar_cb){
        lidar_cb_ = lidar_cb;
    }

    void SetIMUCallback(IMUCallback imu_cb){
        imu_cb_ = imu_cb;
    }

    void SetGPSCallback(GPSCallback gps_cb){
        gps_cb_ = gps_cb;
    }

    void SetFinishCallback(FinshCallback f_cb){
        f_cb_ = f_cb;
    }

    ~SensorDataPlayer(){
        Stop();
    }

private:
    const std::string data_path_;
    rosbag2_cpp::Reader reader_;
    
    std::unique_ptr<rclcpp::Serialization<sensor_msgs::msg::Imu>> imu_serializer_;
    std::unique_ptr<rclcpp::Serialization<sensor_msgs::msg::NavSatFix>> gps_serializer_;
    std::unique_ptr<rclcpp::Serialization<sensor_msgs::msg::PointCloud2>> lidar_serializer_;
    
    const std::string imu_topic_ {"/kitti/imu"};
    const std::string gps_topic_ {"/kitti/gps/fix"};
    const std::string lidar_topic_ {"/kitti/velodyne_points"};

    std::atomic<bool> hold_ {false};
    std::atomic<bool> stop_ {false}; 
    std::condition_variable task_cv_;
    mutable std::mutex loading_mutex_;
    std::thread loading_thread_;

    // cllback function for each sensor source
    LidarCallback lidar_cb_;
    IMUCallback imu_cb_;
    GPSCallback gps_cb_;
    FinshCallback f_cb_;
    
    void LoadingThread();

    void ProcessIMUMsg(rclcpp::SerializedMessage& msg);

    void ProcessLidarMsg(rclcpp::SerializedMessage& msg);

    void ProcessGPSMsg(rclcpp::SerializedMessage& msg);
};




#endif