#include "Persistence.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

#include "Exceptions.h"

namespace logi {
using nlohmann::json;
namespace fs = std::filesystem;

namespace {
void ensureParentDir(const std::string& path) {
    const fs::path parent = fs::path(path).parent_path();
    if (!parent.empty()) fs::create_directories(parent);
}
json toJson(const GeoPoint& p) { return json{{"lat", p.lat}, {"lon", p.lon}}; }
GeoPoint pointFromJson(const json& j) { return {j.at("lat").get<double>(), j.at("lon").get<double>()}; }
}  // namespace

AppConfig Persistence::loadConfig(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw FileError("cannot open config file: " + path);

    json j;
    try {
        in >> j;  // parse the whole file
    } catch (const json::exception& e) {
        throw ParseError(std::string("config JSON is malformed: ") + e.what());
    }

    AppConfig cfg;  // defaults are kept for any missing key
    try {
        if (j.contains("bbox")) {
            const json& b = j.at("bbox");
            cfg.bbox = {b.at("south").get<double>(), b.at("west").get<double>(),
                        b.at("north").get<double>(), b.at("east").get<double>()};
        }
        if (j.contains("depot")) cfg.depot = pointFromJson(j.at("depot"));
        if (j.contains("customers")) {
            cfg.customers.clear();
            for (const auto& c : j.at("customers")) cfg.customers.push_back(pointFromJson(c));
        }
        cfg.truckSpeedKmh = j.value("truckSpeedKmh", cfg.truckSpeedKmh);
        cfg.timeScale = j.value("timeScale", cfg.timeScale);
        cfg.overpassUrl = j.value("overpassUrl", cfg.overpassUrl);
        cfg.networkCachePath = j.value("networkCachePath", cfg.networkCachePath);
    } catch (const json::exception& e) {
        throw ParseError(std::string("config has wrong field types: ") + e.what());
    }
    if (!cfg.bbox.isValid()) throw ParseError("config bbox is not a valid south<north, west<east box");
    return cfg;
}

void Persistence::saveConfig(const AppConfig& cfg, const std::string& path) {
    json j;
    j["bbox"] = {{"south", cfg.bbox.south}, {"west", cfg.bbox.west},
                 {"north", cfg.bbox.north}, {"east", cfg.bbox.east}};
    j["depot"] = toJson(cfg.depot);
    j["customers"] = json::array();
    for (const auto& c : cfg.customers) j["customers"].push_back(toJson(c));
    j["truckSpeedKmh"] = cfg.truckSpeedKmh;
    j["timeScale"] = cfg.timeScale;
    j["overpassUrl"] = cfg.overpassUrl;
    j["networkCachePath"] = cfg.networkCachePath;

    try {
        ensureParentDir(path);
    } catch (const fs::filesystem_error& e) {
        throw FileError(std::string("cannot create directory: ") + e.what());
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) throw FileError("cannot write config file: " + path);
    out << j.dump(2) << '\n';
    if (!out) throw FileError("error while writing config file: " + path);
}

// Cache format (text, one record per line):
//   LOGI_NET 1 <nodeCount> <edgeCount>
//   <lat> <lon>            x nodeCount
//   <a> <b> <meters>       x edgeCount   (each undirected road once, a < b)
void Persistence::saveNetwork(const RoadNetwork& net, const std::string& path) {
    try {
        ensureParentDir(path);
    } catch (const fs::filesystem_error& e) {
        throw FileError(std::string("cannot create directory: ") + e.what());
    }
    std::ofstream out(path, std::ios::trunc);
    if (!out) throw FileError("cannot write network cache: " + path);

    out << "LOGI_NET 1 " << net.nodeCount() << ' ' << net.edgeCount() << '\n';
    out << std::fixed << std::setprecision(8);
    for (RoadNetwork::NodeId i = 0; i < net.nodeCount(); ++i) {
        const GeoPoint& p = net.position(i);
        out << p.lat << ' ' << p.lon << '\n';
    }
    for (RoadNetwork::NodeId u = 0; u < net.nodeCount(); ++u) {
        for (const auto& e : net.neighbors(u)) {
            if (u < e.first) out << u << ' ' << e.first << ' ' << e.second << '\n';
        }
    }
    if (!out) throw FileError("error while writing network cache: " + path);
}

RoadNetwork Persistence::loadNetwork(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw FileError("cannot open network cache: " + path);

    std::string magic;
    int version = 0;
    std::size_t nodes = 0, edges = 0;
    if (!(in >> magic >> version >> nodes >> edges) || magic != "LOGI_NET" || version != 1) {
        throw ParseError("network cache has a bad header: " + path);
    }

    RoadNetwork net;
    try {
        for (std::size_t i = 0; i < nodes; ++i) {
            GeoPoint p;
            if (!(in >> p.lat >> p.lon)) throw ParseError("truncated node list in " + path);
            net.addNode(p);
        }
        for (std::size_t i = 0; i < edges; ++i) {
            std::size_t a = 0, b = 0;
            double w = 0.0;
            if (!(in >> a >> b >> w)) throw ParseError("truncated edge list in " + path);
            net.addRoad(a, b, w);
        }
    } catch (const std::logic_error& e) {  // out_of_range / invalid_argument from RoadNetwork
        throw ParseError(std::string("network cache contains invalid data: ") + e.what());
    }
    return net;
}

}  // namespace logi
