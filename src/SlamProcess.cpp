#include "SlamProcess.hpp"
#include "Backend.hpp"
#include "FrontEnd.hpp"
#include <iostream>


struct SlamProcess::Impl{
    Impl() {
        slam_front_end_.SetKeyFrameCB([this](std::shared_ptr<KeyFrame> key_frame_ptr){
            backend_.PushKeyFrame(key_frame_ptr);
        });

        slam_front_end_.SetEndFrontEndCB([this](){
            backend_.RequestStop();
        });

        slam_front_end_.Start();
    }

    bool IsFinished() const {
        return backend_.IsFinished();
    }

    void Join(){
        slam_front_end_.Join();
        backend_.Join();
    }

    ~Impl(){
        std::cout<<"slam process destructor called\n";
        backend_.RequestStop();
        slam_front_end_.Stop();
        slam_front_end_.Join();
        backend_.Join();
    }
    SlamFrontEnd slam_front_end_;
    Backend backend_;
};

SlamProcess::SlamProcess():process_impl_(std::make_unique<SlamProcess::Impl>()){}

void SlamProcess::Join(){
    process_impl_->Join();
}

bool SlamProcess::IsFinished() const {
    return process_impl_->IsFinished();
}

SlamProcess::~SlamProcess() = default;
