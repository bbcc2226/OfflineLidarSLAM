#ifndef __GEOCONVERTER__
#define __GEOCONVERTER__

#include <Eigen/Dense>
#include <cmath>
#include "DataType.hpp"

class GeoConverter
{
private:
  // Convert WGS84 -> ECEF
  Vec3 Geo2ECEF(double lat, double lon, double alt);

  // Build ECEF->ENU rotation matrix
  Mat3 ECEF2ENU(double lat0, double lon0);

  bool is_initial_ {false};
  Vec3 inital_gps_data_;

public:
  GeoConverter() = default;

  // Convert WGS84 -> ENU
  Vec3 Geo2ENU(Vec3 input_pos);

};



#endif
