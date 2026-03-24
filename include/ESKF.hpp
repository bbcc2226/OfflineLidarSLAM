#ifndef __ESKF__
#define __ESKF__

#include "DataType.hpp"
#include "sophus/se3.hpp"
#include <memory>

class ESKF
{
private:
  struct Impl;
  std::unique_ptr<Impl> eskf_impl_;

public:
  ESKF();

  ESKF(const ESKF & other) = delete;

  ESKF & operator=(const ESKF & other) = delete;

  void Prediction(std::shared_ptr<IMUdata> input_imu);

  void MeasurementUpdateLidar(const Sophus::SE3d& input_pose, const double input_timestamp);

  //void MeasurementUpdateWheel(std::unique_ptr<WheelOdometry> input_wheel);

  Se3 GetLocalizationOutput() const ;

  //bool CheckInitialization() const ;

  ~ESKF();
};


#endif