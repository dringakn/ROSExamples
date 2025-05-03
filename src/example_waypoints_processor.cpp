// test_waypoints_processor.cpp

#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include <string>
#include <map>
#include <filesystem>
#include <algorithm>
#include <limits>

#include "waypoints_processor.hpp"
#include "geodetic_converter.hpp"
#include <nlohmann/json.hpp>

using namespace wp;
using json = nlohmann::json;
using geodetic::GeodeticConverter;

constexpr double REF_LAT  = 49.7939;
constexpr double REF_LON  =  9.9512;
constexpr double REF_ALT  = 10.0;
constexpr double TAKEOFF_HEIGHT    = 10.0;
constexpr double LANDING_HEIGHT    =  0.0;
constexpr bool   INTERPOLATE       = true;
constexpr double INTERPOLATE_DIST  =  5.0;

void printWpList(const std::string &label, const std::vector<Waypoint> &wps) {
    std::cout << "\n" << label << ":\n";
    for (size_t i = 0; i < wps.size(); ++i) {
        const auto &wp = wps[i];
        double yaw_deg = wp.yaw.has_value()
            ? wp.yaw.value() * 180.0/M_PI
            : std::numeric_limits<double>::quiet_NaN();
        std::cout << "  [" << i << "] x=" << wp.x
                  << ", y=" << wp.y
                  << ", z=" << wp.z
                  << ", yaw°=" << yaw_deg
                  << "\n";
    }
}

void printGpsList(
    const std::string &label,
    const std::vector<std::tuple<double,double,double,std::optional<double>>> &gps)
{
    std::cout << "\n" << label << " (lat, lon, alt, yaw°):\n";
    for (size_t i = 0; i < gps.size(); ++i) {
        auto [lat, lon, alt, yaw] = gps[i];
        double yaw_deg = yaw.has_value()
            ? yaw.value() * 180.0/M_PI
            : std::numeric_limits<double>::quiet_NaN();
        std::cout << "  [" << i << "] "
                  << lat << ", " << lon << ", " << alt << ", " << yaw_deg
                  << "\n";
    }
}

void demoHeadingModes() {
    for (auto mode : {"auto","manual","fixed","poi"}) {
        std::cout << "\n=== HEADING MODE: " << mode << " ===\n";

        json params = {
            {"heading_mode", mode},
            {"interpolate_waypoints", INTERPOLATE},
            {"intermediate_waypoint_distance", INTERPOLATE_DIST},
            {"takeoff_height", TAKEOFF_HEIGHT},
            {"landing_height", LANDING_HEIGHT}
        };
        WaypointsProcessor proc(params);

        // 1) Reference & odometry
        proc.setReference(REF_LAT, REF_LON, REF_ALT);
        proc.setOdometry(0.0, 0.0, 0.0, 0.0);

        // 2) Mode-specific setup
        if (std::string(mode) == "poi") {
            proc.setPointOfInterest(REF_LAT+0.0001, REF_LON+0.0001, REF_ALT, "gps");
        }
        if (std::string(mode) == "fixed") {
            proc.setFixedAngleDeg(45.0);
        }

        // 3) Plan a multi-point GPS path
        std::vector<Waypoint> raw_gps = {
            Waypoint(REF_LAT+0.00005, REF_LON+0.00005, REF_ALT+10.0, M_PI/4),
            Waypoint(REF_LAT+0.00010, REF_LON+0.00010, REF_ALT+20.0, M_PI/2)
        };
        auto path = proc.gotoWaypoints(raw_gps, true, "gps");
        printWpList(" Local ENU path", path);
        printGpsList(" GPS output", proc.getWaypointsGps());

        // 4) Single-waypoint in ENU
        Waypoint target_enu(50.0, 50.0, 15.0);
        auto single = proc.gotoWaypoint(target_enu, "enu");
        printWpList(" Single ENU target", single);

        // 5) Vertical moves
        printWpList(" Go to 25 m", proc.gotoHeight(25.0));
        printWpList(" Takeoff",    proc.takeoff());
        printWpList(" Land",       proc.land());

        // 6) Distance & bounds
        double d = proc.getDistanceBetween(path.front(), path.back(), "enu");
        bool inside = proc.checkWithinBounds(
            path,
            Waypoint(-100,-100,-10),
            Waypoint( 100, 100, 100),
            "enu"
        );
        std::cout << "\n Distance first→last: " << d << " m\n";
        std::cout << " All within ±100 m bounds? "
                  << (inside ? "yes" : "no") << "\n";

        // 7) Simulate odometry progression
        std::cout << "\n Simulating progression:\n";
        size_t step = std::max<size_t>(1, path.size()/4);
        for (size_t i = 0; i < path.size(); i += step) {
            const auto &wp = path[i];
            proc.setOdometry(wp.x, wp.y, wp.z, wp.yaw.value_or(0.0));
            std::cout << "  segment → " << proc.getCurrentSegment() << "\n";
        }

        // 8) Abort
        proc.abort();
        std::cout << "\n After abort, waypoints: "
                  << proc.getWaypointsLocal().size() << "\n";
    }
}

void demoCoordinateModes(const std::string &heading="auto") {
    std::cout << "\n=== COORDINATE MODES DEMO (" << heading << " heading) ===\n";
    json params = {
        {"heading_mode", heading},
        {"interpolate_waypoints", INTERPOLATE},
        {"intermediate_waypoint_distance", INTERPOLATE_DIST},
        {"takeoff_height", TAKEOFF_HEIGHT},
        {"landing_height", LANDING_HEIGHT}
    };
    WaypointsProcessor proc(params);
    proc.setReference(REF_LAT, REF_LON, REF_ALT);
    proc.setOdometry(0.0, 0.0, 0.0, 0.0);

    // NED input: (north, east, down)
    std::vector<Waypoint> raw_ned = {
        Waypoint(100.0, 50.0, -20.0),
        Waypoint(150.0, 75.0, -30.0)
    };
    auto path_ned = proc.gotoWaypoints(raw_ned, false, "ned");
    printWpList(" Path from NED input", path_ned);
    printGpsList("  → GPS output", proc.getWaypointsGps());

    // ECEF input: compute via separate converter
    geodetic::GeodeticConverter conv;
    conv.initialiseReference(REF_LAT, REF_LON, REF_ALT);

    double lat   = REF_LAT + 0.0002;
    double lon   = REF_LON + 0.0002;
    double alt_m = 25.0;

    // get back your reference altitude
    auto [lat0, lon0, ref_alt] = proc.getReference();

    // C++ API is geodeticToEcef (returns Eigen::Vector3d)
    Eigen::Vector3d ecef = conv.geodeticToEcef(lat, lon, alt_m + ref_alt);
    double xe = ecef.x(), ye = ecef.y(), ze = ecef.z();

    // build the vector more explicitly
    std::vector<Waypoint> raw_ecef;
    raw_ecef.emplace_back(xe, ye, ze);

    auto path_ecef = proc.gotoWaypoints(raw_ecef, false, "ecef");
    printWpList(" Path from ECEF input", path_ecef);
    printGpsList("  → GPS output", proc.getWaypointsGps());
}

auto makeRectangle = [](double w, double h, double z) {
    std::vector<Waypoint> v;
    v.emplace_back( w/2,  h/2, z);
    v.emplace_back(-w/2,  h/2, z);
    v.emplace_back(-w/2, -h/2, z);
    v.emplace_back( w/2, -h/2, z);
    v.emplace_back( w/2,  h/2, z);  // close loop
    return v;
};

auto makeCircle = [](double r, double z, int segments = 36) {
    std::vector<Waypoint> v;
    for (int i = 0; i <= segments; ++i) {
        double theta = 2*M_PI * i / segments;
        v.emplace_back(r * std::cos(theta), r * std::sin(theta), z);
    }
    return v;
};

auto makeFigure8 = [](double r, double z, int segments = 36) {
    std::vector<Waypoint> v;
    // left loop (CCW)
    for (int i = 0; i <= segments; ++i) {
        double theta = 2*M_PI * i / segments;
        v.emplace_back(-r + r*std::cos(theta),      r*std::sin(theta), z);
    }
    // right loop (CW)
    for (int i = 0; i <= segments; ++i) {
        double theta = 2*M_PI * i / segments;
        v.emplace_back( r + r*std::cos(-theta),     r*std::sin(-theta), z);
    }
    return v;
};

auto makeLawnmower = [](double w, double h, double z, double spacing = 5.0) {
    std::vector<Waypoint> v;
    int rows = std::ceil(h/spacing);
    double y0  = -h/2;
    for (int i = 0; i <= rows; ++i) {
        double y = y0 + i*spacing;
        if (i % 2 == 0) {
            v.emplace_back(-w/2, y, z);
            v.emplace_back( w/2, y, z);
        } else {
            v.emplace_back( w/2, y, z);
            v.emplace_back(-w/2, y, z);
        }
    }
    return v;
};

void demoExportShapes() {
    std::cout << "\n=== EXPORTING ALL SHAPES TO GeoJSON ===\n";
    json params = {
        {"heading_mode", "auto"},
        {"interpolate_waypoints", INTERPOLATE},
        {"intermediate_waypoint_distance", INTERPOLATE_DIST},
        {"takeoff_height", TAKEOFF_HEIGHT},
        {"landing_height", LANDING_HEIGHT}
    };
    WaypointsProcessor proc(params);
    proc.setReference(REF_LAT, REF_LON, REF_ALT);
    proc.setOdometry(0.0, 0.0, 20.0, 0.0);
    double z = TAKEOFF_HEIGHT;


    // waypoint sequences
    std::map<std::string,std::vector<Waypoint>> shapes = {
        {"rectangle", makeRectangle(20.0, 20.0, z)},
        {"circle",    makeCircle   (30.0,  z)},
        {"figure8",   makeFigure8  (10.0,  z)},
        {"lawnmower", makeLawnmower(200.0, 100.0, z, 5.0)}
    };

    std::filesystem::create_directory("exported_shapes");
    for (auto &kv : shapes) {
        const auto &name = kv.first;
        const auto &wps  = kv.second;
        proc.abort();
        // populate internal list (ENU coords, no extra interpolation)
        proc.gotoWaypoints(wps, /*interpolate=*/false, "enu");
        std::string fname = "exported_shapes/" + name + ".geojson";
        proc.exportWaypointsToGeoJSON(fname);
        std::cout << "  • " << name
                  << " → " << fname
                  << " has " << wps.size() << " waypoints\n";
    }
}

int main() {
    demoHeadingModes();
    demoCoordinateModes("auto");
    demoExportShapes();
    return 0;
}
