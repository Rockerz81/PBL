// OverpassClient.h - downloads real road data from OpenStreetMap's Overpass API.
//   libcurl       -> HTTP POST
//   nlohmann/json -> parse the reply and build a RoadNetwork
#pragma once
#include <string>
#include <vector>

#include "RoadNetwork.h"
#include "Types.h"

namespace logi {

struct NamedPlace { std::string name; GeoPoint position; };

class OverpassClient {
public:
    explicit OverpassClient(std::string endpoint = "https://overpass-api.de/api/interpreter",
                            long timeoutSeconds = 120);

    // Overpass QL: all drivable highway ways inside the box, plus their nodes.
    std::string buildQuery(const BoundingBox& box) const;

    // HTTP step only. Throws NetworkError (curl failure or non-200 status).
    std::string fetchRaw(const BoundingBox& box) const;

    // Whole pipeline: fetch -> parse -> keep largest connected component.
    // Throws NetworkError / ParseError.
    RoadNetwork fetchNetwork(const BoundingBox& box) const;

    // Named POIs/roads inside the current map bounds, for readable map labels.
    std::vector<NamedPlace> fetchNamedPlaces(const BoundingBox& box) const;

    // Resolve a user-entered place name to GPS coordinates using Nominatim.
    GeoPoint geocodePlace(const std::string& name, const BoundingBox& box) const;

    // JSON step only (static so it can be tested with a saved response).
    static RoadNetwork parseNetwork(const std::string& jsonText);

private:
    std::string endpoint_;
    long timeoutSeconds_;
};

}  // namespace logi
