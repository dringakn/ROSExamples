#include <ros/ros.h>
#include "geodetic_converter.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_gps_ned");
    ros::NodeHandle nh;

    geodetic::GeodeticConverter conv;
    // List of GPS waypoints: {lat, lon, alt}
    std::vector<std::tuple<double, double, double>> gps_wps = {
        {48.0, 11.0, 500.0},
        {48.0005, 11.0003, 505.0},
        {48.0010, 11.0006, 502.0}};

    // Use first waypoint as reference
    // double lat0, lon0, alt0;
    // std::tie(lat0, lon0, alt0) = gps_wps.front(); // Assuming C++11
    auto [lat0, lon0, alt0] = gps_wps.front(); // Assuming C++17 or later
    conv.initialiseReference(lat0, lon0, alt0);

    ROS_INFO("Converting %zu waypoints:", gps_wps.size());
    std::vector<Eigen::Vector3d> ned_wps;
    for (auto &wp : gps_wps)
    {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = wp;
        auto ned = conv.geodeticToNed(lat, lon, alt);
        ned_wps.push_back(ned);
        ROS_INFO(" GPS(%.6f,%.6f,%.1f) -> NED(%.3f, %.3f, %.3f)",
                 lat, lon, alt, ned.x(), ned.y(), ned.z());
    }

    for (auto &ned : ned_wps)
    {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = conv.nedToGeodetic(ned.x(), ned.y(), ned.z());
        ROS_INFO(" NED(%.3f, %.3f, %.3f) -> GPS(%.6f,%.6f,%.1f)",
                 ned.x(), ned.y(), ned.z(), lat, lon, alt);
    }

    // GPS to ENU conversion
    std::vector<Eigen::Vector3d> enu_wps;
    for (auto &wp : gps_wps)
    {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = wp;
        auto enu = conv.geodeticToEnu(lat, lon, alt);
        enu_wps.push_back(enu);
        ROS_INFO(" GPS(%.6f,%.6f,%.1f) -> ENU(%.3f, %.3f, %.3f)",
                 lat, lon, alt, enu.x(), enu.y(), enu.z());
    }
    for (auto &enu : enu_wps)
    {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = conv.enuToGeodetic(enu.x(), enu.y(), enu.z());
        ROS_INFO(" ENU(%.2f, %.2f, %.2f) -> GPS(%.6f,%.6f,%.1f)",
                 enu.x(), enu.y(), enu.z(), lat, lon, alt);
    }

    // GPS to ECEF conversion
    std::vector<Eigen::Vector3d> ecef_wps;
    for (auto &wp : gps_wps)
    {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = wp;
        auto ecef = conv.geodeticToEcef(lat, lon, alt);
        ecef_wps.push_back(ecef);
        ROS_INFO(" GPS(%.6f,%.6f,%.1f) -> ECEF(%.1f, %.1f, %.1f)",
                 lat, lon, alt, ecef.x(), ecef.y(), ecef.z());
    }
    for (auto &ecef : ecef_wps)
    {
        double lat, lon, alt;
        std::tie(lat, lon, alt) = conv.ecefToGeodetic(ecef.x(), ecef.y(), ecef.z());
        ROS_INFO(" ECEF(%.1f, %.1f, %.1f) -> GPS(%.6f,%.6f,%.1f)",
                 ecef.x(), ecef.y(), ecef.z(), lat, lon, alt);
    }
    return 0;
}
