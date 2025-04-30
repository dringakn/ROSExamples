// geodetic_converter.hpp
#pragma once
#include <cmath>
#include <tuple>
#include <stdexcept>
#include <Eigen/Dense>

namespace geodetic {

class GeodeticConverter
{
public:
  GeodeticConverter()
  : have_reference_(false) {}

  bool isInitialised() const { return have_reference_; }

  std::tuple<double,double,double> getReference() const {
    if (!have_reference_) throw std::runtime_error{"Reference not set"};
    return {lat0_, lon0_, alt0_};
  }

  void initialiseReference(double lat_deg, double lon_deg, double alt_m) {
    lat0_ = deg2rad(lat_deg);
    lon0_ = deg2rad(lon_deg);
    alt0_ = alt_m;

    ecef0_ = geodeticToEcef(lat_deg, lon_deg, alt_m);

    // ECEF→NED & NED→ECEF
    ecef_to_ned_ = ecefToNeu(lat0_, lon0_);
    ned_to_ecef_ = ecef_to_ned_.transpose();

    have_reference_ = true;
  }

  // --- Conversions ---
  Eigen::Vector3d geodeticToEcef(double lat_deg, double lon_deg, double alt) const {
    double lat = deg2rad(lat_deg), lon = deg2rad(lon_deg);
    double xi = std::sqrt(1 - e2_*std::sin(lat)*std::sin(lat));
    double rn = a_/xi + alt;
    return {
      rn * std::cos(lat)*std::cos(lon),
      rn * std::cos(lat)*std::sin(lon),
      (a_/xi*(1-e2_) + alt)*std::sin(lat)
    };
  }

  std::tuple<double,double,double> ecefToGeodetic(double x,double y,double z) const {
    // Zhu (1994)
    double r = std::hypot(x,y);
    double Esq = a_*a_ - b_*b_;
    double F = 54*b_*b_*z*z;
    double G = r*r + (1-e2_)*z*z - e2_*Esq;
    double C = (e2_*e2_*F*r*r)/(G*G*G);
    double S = std::cbrt(1 + C + std::sqrt(C*C+2*C));
    double P = F/(3*(S+1/S+1)*(S+1/S+1)*G*G);
    double Q = std::sqrt(1+2*e2_*e2_*P);
    double r0 = -(P*e2_*r)/(1+Q) +
      std::sqrt(0.5*a_*a_*(1+1/Q) - P*(1-e2_)*z*z/(Q*(1+Q)) - 0.5*P*r*r);
    double U = std::sqrt((r-e2_*r0)*(r-e2_*r0) + z*z);
    double V = std::sqrt((r-e2_*r0)*(r-e2_*r0) + (1-e2_)*z*z);
    double Z0 = b_*b_*z/(a_*V);
    double h = U*(1 - b_*b_/(a_*V));
    double lat = std::atan((z + e2_2*Z0)/r);
    double lon = std::atan2(y,x);
    return { rad2deg(lat), rad2deg(lon), h };
  }

  Eigen::Vector3d ecefToNed(double x,double y,double z) const {
    if (!have_reference_) throw std::runtime_error{"init ref first"};
    Eigen::Vector3d v{x,y,z};
    auto d = v - ecef0_;
    auto ned = ecef_to_ned_ * d;
    return { ned(0), ned(1), -ned(2) };
  }

  Eigen::Vector3d nedToEcef(double north,double east,double down) const {
    if (!have_reference_) throw std::runtime_error{"init ref first"};
    Eigen::Vector3d n{north, east, -down};
    return ned_to_ecef_ * n + ecef0_;
  }

  Eigen::Vector3d geodeticToNed(double lat,double lon,double alt) {
    auto ecef = geodeticToEcef(lat,lon,alt);
    return ecefToNed(ecef.x(), ecef.y(), ecef.z());
  }

  std::tuple<double,double,double> nedToGeodetic(double north,double east,double down) {
    auto ecef = nedToEcef(north,east,down);
    return ecefToGeodetic(ecef.x(), ecef.y(), ecef.z());
  }

  Eigen::Vector3d geodeticToEnu(double lat,double lon,double alt) {
    auto n = geodeticToNed(lat,lon,alt);
    return { n.y(), n.x(), -n.z() };
  }

  std::tuple<double,double,double> enuToGeodetic(double east,double north,double up) {
    return nedToGeodetic(north, east, -up);
  }

private:
  static Eigen::Matrix3d ecefToNeu(double lat, double lon) {
    double sLat=std::sin(lat), cLat=std::cos(lat);
    double sLon=std::sin(lon), cLon=std::cos(lon);
    Eigen::Matrix3d R;
    R << -sLat*cLon, -sLat*sLon,  cLat,
         -sLon,       cLon,       0.0,
          cLat*cLon,   cLat*sLon,  sLat;
    return R;
  }

  static double deg2rad(double d) { return d*M_PI/180.0; }
  static double rad2deg(double r) { return r*180.0/M_PI; }

  inline static constexpr double a_ = 6378137.0;
  inline static constexpr double b_ = 6356752.314245;
  inline static constexpr double e2_ = 6.69437999014e-3;
  inline static constexpr double e2_2 = 6.73949674228e-3;

  bool have_reference_;
  double lat0_, lon0_, alt0_;
  Eigen::Vector3d ecef0_;
  Eigen::Matrix3d ecef_to_ned_, ned_to_ecef_;
};

} // namespace geodetic
