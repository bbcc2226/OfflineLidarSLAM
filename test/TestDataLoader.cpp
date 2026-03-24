#include <gtest/gtest.h>
#include <iostream>
#include <string>
#include "DataLoader.hpp"

TEST(DataLaoderTest, Testplayer)
{
    SensorDataPlayer loader("/root/ros2_ws/2011_09_26_drive_0035_extract_bag");
    loader.SetIMUCallback([](std::shared_ptr<IMUdata> imu){
        std::cout<< "Processing imu message: "<<imu->timestamp_<<"\n";
    });
    loader.SetGPSCallback([](std::shared_ptr<GPSdata> gps){
        std::cout<< "Processing gps message "<<gps->timestamp_<<"\n";
    });
    loader.SetLidarCallback([](std::shared_ptr<PointCloud> cloud){
        std::cout<< "Processing cloud message "<<cloud->timestamp_<<"\n";
    });
    loader.SetFinishCallback([](){
        std::cout<<"loading process is done \n";
    });
    loader.Start();
    std::this_thread::sleep_for(std::chrono::seconds(5));

    loader.Pause();

    std::this_thread::sleep_for(std::chrono::seconds(3));

    loader.Resume();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    loader.Stop();


}