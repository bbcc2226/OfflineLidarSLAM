#include "GeoConverter.hpp"
#include "ConfigManager.hpp"

Vec3 GeoConverter::Geo2ECEF(double lat, double lon, double alt)
{

  lat = lat * M_PI / 180.0;
  lon = lon * M_PI / 180.0;

  const double N = ConfigManager::Get().GeoConverter_.a / std::sqrt(
    1 - ConfigManager::Get().GeoConverter_.e2 * std::sin(
      lat) * std::sin(lat));
  const double x = (N + alt) * std::cos(lat) * std::cos(lon);
  const double y = (N + alt) * std::cos(lat) * std::sin(lon);
  const double z = (N * (1 - ConfigManager::Get().GeoConverter_.e2) + alt) * std::sin(lat);

  return Vec3(x, y, z);
}

Mat3 GeoConverter::ECEF2ENU(double lat0, double lon0)
{

  lat0 = lat0 * M_PI / 180.0;
  lon0 = lon0 * M_PI / 180.0;

  Mat3 R;
  R << -std::sin(lon0), std::cos(lon0), 0,
    -std::sin(lat0) * std::cos(lon0), -std::sin(lat0) * std::sin(lon0), std::cos(lat0),
    std::cos(lat0) * std::cos(lon0), std::cos(lat0) * std::sin(lon0), std::sin(lat0);

  return R;
}

Vec3 GeoConverter::Geo2ENU(Vec3 input_pos)
{
  if(!is_initial_){
    is_initial_ = true;
    inital_gps_data_ = input_pos;
    return Vec3::Zero();
  }
  const Vec3 ecef = Geo2ECEF(input_pos[0],input_pos[1],input_pos[2]);
  const Vec3 ecef_ref = Geo2ECEF(inital_gps_data_[0], inital_gps_data_[1], inital_gps_data_[2]);
  const Mat3 R = ECEF2ENU(inital_gps_data_[0], inital_gps_data_[1]);

  return R * (ecef - ecef_ref);

}

