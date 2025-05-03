#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <tuple>
#include <optional>
#include <unordered_map>
#include <functional>
#include <stdexcept>
#include <fstream>

#include "geodetic_converter.hpp"
#include <nlohmann/json.hpp>          // https://github.com/nlohmann/json

using geodetic::GeodeticConverter;

namespace wp {

struct Waypoint {
    double x, y, z;
    std::optional<double> yaw;
    Waypoint(double _x=0, double _y=0, double _z=0, std::optional<double> _yaw = std::nullopt)
      : x(_x), y(_y), z(_z), yaw(_yaw) {}
};

class WaypointsProcessor {
public:
    WaypointsProcessor(const nlohmann::json &params) {
        initState();
        loadParameters(params);
        buildCoordMaps();
    }

    // ─ Public API ────────────────────────────────────────────────────────

    void setReference(double lat, double lon, double alt) {
        gc.initialiseReference(lat, lon, alt);
        reference_altitude = alt;
    }

    std::tuple<double,double,double> getReference() {
        auto [lat0, lon0, alt0] = gc.getReference();
        reference_altitude = alt0;
        return {lat0, lon0, alt0};
    }

    void setOdometry(double x, double y, double z, double yaw_rad) {
        odometry = Waypoint(x,y,z,yaw_rad);
        updateCurrentSegment(x,y,z);
    }

    Waypoint getOdometry() const { return odometry; }

    void setPointOfInterest(double a, double b, double c, const std::string &coord) {
        auto [lx,ly,lz] = to_local( Waypoint(a,b,c), coord );
        poi = Waypoint(lx,ly,lz);
    }

    std::tuple<double,double,double> getPointOfInterest(const std::string &coord) const {
        auto w = from_local(poi.x, poi.y, poi.z, coord);
        return {w.x, w.y, w.z};
    }

    void setFixedAngleDeg(double deg) {
        fixed_angle = deg * M_PI/180.0;
    }

    double getFixedAngleDeg() const {
        return fixed_angle * 180.0/M_PI;
    }

    double getTotalDistance() const { return total_distance; }
    size_t getCurrentSegment() const { return current_segment; }
    const std::vector<Waypoint>& getWaypointsLocal() const { return waypoints; }

    // GPS output: (lat, lon, alt, yaw)
    std::vector<std::tuple<double,double,double,std::optional<double>>>
    getWaypointsGps() const {
        std::vector<std::tuple<double,double,double,std::optional<double>>> out;
        for (auto &w: waypoints) {
            auto [lat,lon,alt] = gc.enuToGeodetic(w.x,w.y,w.z);
            out.emplace_back(lat, lon, alt, w.yaw);
        }
        return out;
    }

    double getDistanceBetween(const Waypoint &a, const Waypoint &b, const std::string &coord) const {
        auto [x1,y1,z1] = to_local(a, coord);
        auto [x2,y2,z2] = to_local(b, coord);
        return std::hypot(x2-x1, y2-y1, z2-z1);
    }

    void reset() {
        waypoints.clear();
        current_segment = 0;
        total_distance = 0.0;
    }

    std::vector<Waypoint> gotoWaypoints(
        const std::vector<Waypoint> &raw_wps,
        bool liftoff,
        const std::string &coord)
    {
        if (raw_wps.empty()) return {};
        reset();
        waypoints.push_back(odometry);
        // vertical hop
        if (liftoff) {
            auto [tx,ty,tz] = to_local(raw_wps[0], coord);
            double yaw = selectYaw(odometry, tx, ty, raw_wps[0].yaw);
            waypoints.emplace_back(odometry.x, odometry.y, tz, yaw);
        }
        // traverse
        for (auto &rw: raw_wps) {
            auto [tx,ty,tz] = to_local(rw, coord);
            auto &prev = waypoints.back();
            double yaw = selectYaw(prev, tx, ty, rw.yaw);
            waypoints.emplace_back(tx, ty, tz, yaw);
        }
        if (interpolate_waypoints) interpolate();
        computeTotalDistance();
        return waypoints;
    }

    std::vector<Waypoint> gotoWaypoint(const Waypoint &wp, const std::string &coord) {
        return gotoWaypoints({wp}, false, coord);
    }

    std::vector<Waypoint> gotoHeight(double h) {
        Waypoint tgt{odometry.x, odometry.y, h};
        return gotoWaypoint(tgt, "enu");
    }

    std::vector<Waypoint> takeoff() {
        return gotoHeight(odometry.z + takeoff_height);
    }

    std::vector<Waypoint> land() {
        return gotoHeight(landing_height);
    }

    void abort() { reset(); }

    bool checkWithinBounds(
        const std::vector<Waypoint> &wps,
        const Waypoint &min_b,
        const Waypoint &max_b,
        const std::string &coord) const
    {
        auto [minx,miny,minz] = to_local(min_b, coord);
        auto [maxx,maxy,maxz] = to_local(max_b, coord);
        for (auto &w: wps) {
            auto [x,y,z] = to_local(w, coord);
            if (x<minx||x>maxx || y<miny||y>maxy || z<minz||z>maxz)
                return false;
        }
        return true;
    }

    // ─ GeoJSON I/O ──────────────────────────────────────────────────

    std::vector<Waypoint> importWaypointsFromGeoJSON(const std::string &path,
                                                     const std::string &coord="gps")
    {
        std::ifstream f(path);
        nlohmann::json doc; 
        f >> doc;
        if (doc.value("type","") != "FeatureCollection")
            throw std::runtime_error("GeoJSON must be a FeatureCollection");
        std::vector<Waypoint> loaded;
        for (auto &feat: doc["features"]) {
            if (feat["geometry"]["type"] != "Point") continue;
            auto coords = feat["geometry"]["coordinates"];
            double lon = coords[0], lat = coords[1];
            double alt = coords.size()>2 ? coords[2].get<double>() : 0.0;
            std::optional<double> yaw = std::nullopt;
            if (feat["properties"].contains("yaw"))
                yaw = feat["properties"]["yaw"].get<double>();
            Waypoint raw = (coord=="gps")
                ? Waypoint(lat, lon, alt, yaw)
                : Waypoint(coords[0], coords[1], alt, yaw);
            auto [ex,ey,ez] = to_local(raw, coord);
            loaded.emplace_back(ex,ey,ez,yaw);
        }
        waypoints = loaded;
        computeTotalDistance();
        return waypoints;
    }

    void exportWaypointsToGeoJSON(const std::string &path) const {
        nlohmann::json doc;
        doc["type"] = "FeatureCollection";
        doc["features"] = nlohmann::json::array();
        for (auto &[lat,lon,alt,yaw] : getWaypointsGps()) {
            nlohmann::json feat;
            feat["type"] = "Feature";
            feat["geometry"] = { {"type","Point"},
                                 {"coordinates",{lon,lat,alt}} };
            feat["properties"] = { {"yaw", yaw.value_or(0.0)} };
            doc["features"].push_back(std::move(feat));
        }
        std::ofstream f(path);
        f << doc.dump(2);
    }

private:
    mutable GeodeticConverter gc;
    std::vector<Waypoint> waypoints;
    Waypoint odometry, poi;
    double fixed_angle{}, reference_altitude{};
    std::string heading_mode;
    bool interpolate_waypoints{};
    double intermediate_waypoint_distance{}, takeoff_height{}, landing_height{};
    size_t current_segment{};
    double total_distance{}, distance_tolerance{0.1};

    using ToLocalFn  = std::function<std::tuple<double,double,double>(const Waypoint&)>;
    using FromLocalFn= std::function<Waypoint(double,double,double)>;
    std::unordered_map<std::string,ToLocalFn>   to_map;
    std::unordered_map<std::string,FromLocalFn> from_map;

    void initState() {
        waypoints.clear();
        odometry = Waypoint{};
        poi      = Waypoint{};
        fixed_angle = reference_altitude = 0.0;
        heading_mode = "auto";
        interpolate_waypoints = false;
        intermediate_waypoint_distance = 1.0;
        takeoff_height = landing_height = 0.0;
        current_segment = 0;
        total_distance = 0.0;
        distance_tolerance = 0.1;
    }

    void loadParameters(const nlohmann::json &p) {
        static const std::vector<std::string> req = {
            "heading_mode","interpolate_waypoints",
            "intermediate_waypoint_distance",
            "takeoff_height","landing_height"
        };
        for (auto &k: req)
            if (!p.contains(k))
                throw std::invalid_argument("Missing parameter: "+k);
        heading_mode = p["heading_mode"].get<std::string>();
        if (heading_mode!="auto"&&heading_mode!="manual"&&
            heading_mode!="fixed"&&heading_mode!="poi")
            throw std::invalid_argument("Invalid heading_mode");
        intermediate_waypoint_distance = p["intermediate_waypoint_distance"].get<double>();
        if (intermediate_waypoint_distance<=0)
            throw std::invalid_argument("intermediate_waypoint_distance must be >0");
        takeoff_height = p["takeoff_height"].get<double>();
        landing_height = p["landing_height"].get<double>();
        if (takeoff_height<0||landing_height<0)
            throw std::invalid_argument("takeoff/landing heights must be >=0");
        interpolate_waypoints = p["interpolate_waypoints"].get<bool>();
    }

    void buildCoordMaps() {
        to_map = {
          {"gps", [this](auto &w){
              auto v = gc.geodeticToEnu(w.x, w.y, w.z + reference_altitude);
              return std::make_tuple(v.x(), v.y(), v.z());
          }},
          {"enu", [](auto &w){
              return std::make_tuple(w.x, w.y, w.z);
          }},
          {"ned", [](auto &w){
              // north=w.y, east=w.x, down=–w.z → matches Python’s (wp.y,wp.x,-wp.z)
              return std::make_tuple(w.y, w.x, -w.z);
          }},
          {"ecef", [this](auto &w){
              // geodetic::ecefToNed ⇒ (north, east, down)
              auto v = gc.ecefToNed(w.x, w.y, w.z);
              // to ENU: (east, north, up)
              return std::make_tuple(v.y(), v.x(), -v.z());
          }}
        };
      
        from_map = {
          {"gps", [this](double x,double y,double z){
              auto [lat,lon,abs] = gc.enuToGeodetic(x,y,z);
              return Waypoint{lat, lon, abs - reference_altitude};
          }},
          {"enu", [](double x,double y,double z){
              return Waypoint{x,y,z};
          }},
          {"ned", [](double x,double y,double z){
              return Waypoint{y, x, -z};
          }},
          {"ecef", [this](double x,double y,double z){
              // local x=east, y=north, z=up → down = -up
              auto v = gc.nedToEcef(y, x, -z);
              return Waypoint{v.x(), v.y(), v.z()};
          }}
        };
    }

    std::tuple<double,double,double> to_local(const Waypoint &w, const std::string &c) const {
        if (!to_map.count(c)) throw std::invalid_argument("Unsupported coord: "+c);
        return to_map.at(c)(w);
    }

    Waypoint from_local(double x,double y,double z,const std::string &c) const {
        if (!from_map.count(c)) throw std::invalid_argument("Unsupported coord: "+c);
        return from_map.at(c)(x,y,z);
    }

    static double normalizeYaw(double yaw) {
        return std::atan2(std::sin(yaw), std::cos(yaw));
    }

    double selectYaw(const Waypoint &prev, double tx, double ty,
                     const std::optional<double> &downstream) const
    {
        double raw = 0;
        if (heading_mode=="auto") {
            raw = std::atan2(ty-prev.y, tx-prev.x);
        } else if (heading_mode=="fixed") {
            raw = fixed_angle;
        } else if (heading_mode=="poi") {
            raw = std::atan2(poi.y - prev.y, poi.x - prev.x);
        } else /* manual */ {
            if (downstream) raw = *downstream;
            else if (prev.yaw) raw = *prev.yaw;
            else raw = odometry.yaw.value_or(0.0);
        }
        return normalizeYaw(raw);
    }

    void computeTotalDistance() {
        total_distance = 0.0;
        for (size_t i=1; i<waypoints.size(); ++i) {
            auto &a = waypoints[i-1], &b = waypoints[i];
            total_distance += std::hypot(b.x-a.x, b.y-a.y, b.z-a.z);
        }
    }

    void updateCurrentSegment(double x, double y, double z, double thr=0.5) {
        double thr2 = thr*thr;
        while (current_segment + 1 < waypoints.size()) {
            auto &w = waypoints[current_segment+1];
            if ( std::pow(w.x-x,2)+std::pow(w.y-y,2)+std::pow(w.z-z,2) < thr2 )
                ++current_segment;
            else break;
        }
    }

    void interpolate() {
        if (waypoints.size()<2) return;
        std::vector<Waypoint> out;
        out.push_back(waypoints.front());
        double sep = intermediate_waypoint_distance;
        for (size_t i=1; i<waypoints.size(); ++i) {
            Waypoint a = out.back(), b = waypoints[i];
            double dist = std::hypot(b.x-a.x,b.y-a.y,b.z-a.z);
            while (dist > sep + distance_tolerance) {
                double frac = sep / dist;
                double nx = a.x + frac*(b.x-a.x);
                double ny = a.y + frac*(b.y-a.y);
                double nz = a.z + frac*(b.z-a.z);
                double nyaw = selectYaw(a, b.x, b.y, b.yaw);
                out.emplace_back(nx,ny,nz,nyaw);
                a = out.back();
                dist = std::hypot(b.x-a.x,b.y-a.y,b.z-a.z);
            }
            out.push_back(b);
        }
        waypoints.swap(out);
    }
};

} // namespace wp
