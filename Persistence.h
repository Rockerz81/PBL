// Persistence.h - file-based save/load using <fstream>.
//   * AppConfig  -> human-readable JSON   (config/config.json)
//   * RoadNetwork -> compact text cache   (data/network_cache.txt) so the
//     Overpass API is hit once, not on every launch.
#pragma once
#include <string>
#include <vector>

#include "RoadNetwork.h"
#include "Types.h"

namespace logi {

struct AppConfig {
    BoundingBox bbox{30.31, 78.02, 30.34, 78.06};  // central Dehradun (~3 km x 4 km)
    GeoPoint depot{30.3256, 78.0437};              // near Clock Tower, Dehradun
    std::vector<GeoPoint> customers;               // empty => app picks random ones
    double truckSpeedKmh{40.0};
    double timeScale{25.0};
    std::string overpassUrl{"https://overpass-api.de/api/interpreter"};
    std::string networkCachePath{"data/network_cache.txt"};
};

class Persistence {
public:
    // Throw FileError (cannot open/write) or ParseError (bad content).
    static AppConfig loadConfig(const std::string& path);
    static void saveConfig(const AppConfig& cfg, const std::string& path);

    static void saveNetwork(const RoadNetwork& net, const std::string& path);
    static RoadNetwork loadNetwork(const std::string& path);
};

}  // namespace logi
