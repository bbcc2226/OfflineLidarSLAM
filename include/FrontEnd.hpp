#ifndef __FRONTEND__
#define __FRONTEND__

#include <memory>
#include <functional>
#include "DataType.hpp"

// Front-end slam is loosely LIO
// contain:
// 1. Lidar Odometry(NDT based)
// 2. DataLoader with ros2 bag
// 3. ESKF state estimator

class SlamFrontEnd{
public:
    using KFCallback = std::function<void(std::shared_ptr<KeyFrame>)>;
    using EndFrontEndCallback = std::function<void()>;

    SlamFrontEnd();

    ~SlamFrontEnd();

    SlamFrontEnd(const SlamFrontEnd & other) = delete;

    SlamFrontEnd& operator=(const SlamFrontEnd & other) = delete;

    void Start();
    
    void Stop();

    void Join();

    void SetKeyFrameCB(KFCallback kf_cb);

    void SetEndFrontEndCB(EndFrontEndCallback fe_cb);

private:
    struct Impl;
    std::unique_ptr<Impl> frontend_impl_;

};


#endif