#include "OverpassClient.h"

#include <curl/curl.h>

#include <algorithm>

#include <iomanip>
#include <cctype>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <unordered_map>

#include "Exceptions.h"

namespace logi {
using nlohmann::json;

namespace {

// libcurl calls this for every chunk of the response body.
std::size_t writeCallback(char* data, std::size_t size, std::size_t count, void* userdata) {
    auto* body = static_cast<std::string*>(userdata);
    body->append(data, size * count);
    return size * count;  // returning less than this would abort the transfer
}

// RAII: curl_global_init once per process, cleanup at exit.
struct CurlGlobal {
    CurlGlobal() { curl_global_init(CURL_GLOBAL_DEFAULT); }
    ~CurlGlobal() { curl_global_cleanup(); }
};

// RAII: easy handle is always released, even when we throw.
using CurlHandle = std::unique_ptr<CURL, decltype(&curl_easy_cleanup)>;

}  // namespace

OverpassClient::OverpassClient(std::string endpoint, long timeoutSeconds)
    : endpoint_(std::move(endpoint)), timeoutSeconds_(timeoutSeconds) {}

std::string OverpassClient::buildQuery(const BoundingBox& b) const {
    if (!b.isValid()) throw std::invalid_argument("OverpassClient: invalid bounding box");
    std::ostringstream q;
    q << std::fixed << std::setprecision(6);
    // `out skel qt` = geometry only (no tags), quad-tile sorted: smallest reply.
    q << "[out:json][timeout:90];"
      << "way[\"highway\"~\"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|"
         "living_street|motorway_link|trunk_link|primary_link|secondary_link|tertiary_link)$\"]("
      << b.south << ',' << b.west << ',' << b.north << ',' << b.east << ");"
      << "(._;>;);"  // add every node referenced by those ways
      << "out skel qt;";
    return q.str();
}

std::string OverpassClient::fetchRaw(const BoundingBox& box) const {
    static CurlGlobal globalInit;

    CurlHandle curl(curl_easy_init(), &curl_easy_cleanup);
    if (!curl) throw NetworkError("curl_easy_init failed");

    // Overpass expects a form field named "data" holding the query.
    std::unique_ptr<char, decltype(&curl_free)> escaped(nullptr, &curl_free);
    const std::string query = buildQuery(box);
    escaped.reset(curl_easy_escape(curl.get(), query.c_str(), static_cast<int>(query.size())));
    if (!escaped) throw NetworkError("curl_easy_escape failed");
    const std::string postBody = std::string("data=") + escaped.get();

    std::string response;
    curl_easy_setopt(curl.get(), CURLOPT_URL, endpoint_.c_str());
    curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDS, postBody.c_str());
    curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl.get(), CURLOPT_USERAGENT, "LogisticsRouter/1.0 (student project)");
    curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, timeoutSeconds_);
    curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_ACCEPT_ENCODING, "");  // let curl negotiate gzip

    const CURLcode rc = curl_easy_perform(curl.get());
    if (rc != CURLE_OK) {
        throw NetworkError(std::string("HTTP request failed: ") + curl_easy_strerror(rc));
    }
    long status = 0;
    curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &status);
    if (status != 200) {
        throw NetworkError("Overpass returned HTTP " + std::to_string(status) +
                           (status == 429 ? " (rate limited - wait a minute)" : ""));
    }
    return response;
}

RoadNetwork OverpassClient::fetchNetwork(const BoundingBox& box) const {
    return parseNetwork(fetchRaw(box));
}

std::vector<NamedPlace> OverpassClient::fetchNamedPlaces(const BoundingBox& b) const {
    static CurlGlobal globalInit;
    if (!b.isValid()) throw std::invalid_argument("OverpassClient: invalid bounding box");

    // Overpass instances can temporarily return HTTP 429/502/504. Try the
    // configured server first, then public mirrors instead of giving up after
    // one failed request. The road-network cache remains independent of labels.
    std::vector<std::string> endpoints{
        endpoint_,
        "https://overpass.kumi.systems/api/interpreter",
        "https://overpass.private.coffee/api/interpreter",
        "https://overpass-api.de/api/interpreter"
    };
    std::vector<std::string> uniqueEndpoints;
    for (const auto& endpoint : endpoints) {
        if (!endpoint.empty() && std::find(uniqueEndpoints.begin(), uniqueEndpoints.end(), endpoint) == uniqueEndpoints.end())
            uniqueEndpoints.push_back(endpoint);
    }

    std::ostringstream q;
    q << std::fixed << std::setprecision(6)
      << "[out:json][timeout:35];("
      << "node[\"name\"](" << b.south << ',' << b.west << ',' << b.north << ',' << b.east << ");"
      << "node[\"amenity\"][\"name\"](" << b.south << ',' << b.west << ',' << b.north << ',' << b.east << ");"
      << "way[\"name\"][\"highway\"](" << b.south << ',' << b.west << ',' << b.north << ',' << b.east << ");"
      << ");out center tags;";
    const std::string query = q.str();

    std::string lastError = "No Overpass endpoints were available.";
    for (const auto& endpoint : uniqueEndpoints) {
        CurlHandle curl(curl_easy_init(), &curl_easy_cleanup);
        if (!curl) throw NetworkError("curl_easy_init failed");
        std::unique_ptr<char, decltype(&curl_free)> escaped(
            curl_easy_escape(curl.get(), query.c_str(), static_cast<int>(query.size())), &curl_free);
        if (!escaped) throw NetworkError("curl_easy_escape failed");
        const std::string postBody = std::string("data=") + escaped.get();
        std::string response;
        curl_easy_setopt(curl.get(), CURLOPT_URL, endpoint.c_str());
        curl_easy_setopt(curl.get(), CURLOPT_POSTFIELDS, postBody.c_str());
        curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl.get(), CURLOPT_USERAGENT, "LogisticsRouter/1.1 (student project; map labels)");
        curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, 40L);
        curl_easy_setopt(curl.get(), CURLOPT_CONNECTTIMEOUT, 10L);
        curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl.get(), CURLOPT_ACCEPT_ENCODING, "");

        const CURLcode rc = curl_easy_perform(curl.get());
        if (rc != CURLE_OK) {
            lastError = endpoint + ": " + curl_easy_strerror(rc);
            continue;
        }
        long status = 0;
        curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &status);
        if (status != 200) {
            lastError = endpoint + ": HTTP " + std::to_string(status);
            continue;
        }

        json doc;
        try { doc = json::parse(response); }
        catch (const json::exception& e) {
            lastError = endpoint + ": invalid JSON response (" + e.what() + ")";
            continue;
        }

        std::vector<NamedPlace> places;
        std::unordered_map<std::string, bool> seen;
        for (const auto& el : doc.value("elements", json::array())) {
            if (!el.contains("tags") || !el["tags"].is_object()) continue;
            const std::string placeName = el["tags"].value("name", std::string{});
            if (placeName.empty()) continue;
            double lat = 0.0, lon = 0.0;
            if (el.contains("lat") && el.contains("lon")) {
                lat = el["lat"].get<double>();
                lon = el["lon"].get<double>();
            } else if (el.contains("center")) {
                lat = el["center"].value("lat", 0.0);
                lon = el["center"].value("lon", 0.0);
            }
            if (lat < b.south || lat > b.north || lon < b.west || lon > b.east) continue;
            const std::string key = placeName + "@" + std::to_string(lat).substr(0, 9) + "," +
                                    std::to_string(lon).substr(0, 9);
            if (seen.emplace(key, true).second) places.push_back({placeName, {lat, lon}});
        }
        if (!places.empty()) return places;
        lastError = endpoint + ": no named places were returned for this map area";
    }
    throw NetworkError("Place labels unavailable after trying Overpass mirrors (" + lastError + "). Check internet or retry later.");
}

GeoPoint OverpassClient::geocodePlace(const std::string& name, const BoundingBox& b) const {
    static CurlGlobal globalInit;
    CurlHandle curl(curl_easy_init(), &curl_easy_cleanup);
    if (!curl) throw NetworkError("curl_easy_init failed");
    // Encode the user text as a query parameter.
    std::string query = name + ", Dehradun, Uttarakhand, India";
    std::unique_ptr<char, decltype(&curl_free)> escaped(
        curl_easy_escape(curl.get(), query.c_str(), static_cast<int>(query.size())), &curl_free);
    if (!escaped) throw NetworkError("curl_easy_escape failed");
    const std::string url = "https://nominatim.openstreetmap.org/search?format=jsonv2&limit=5&viewbox=" +
        std::to_string(b.west) + "," + std::to_string(b.north) + "," +
        std::to_string(b.east) + "," + std::to_string(b.south) + "&bounded=1&q=" + escaped.get();
    std::string response;
    curl_easy_setopt(curl.get(), CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl.get(), CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, writeCallback);
    curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response);
    curl_easy_setopt(curl.get(), CURLOPT_USERAGENT, "LogisticsRouter/1.0 (student project; contact: student)");
    curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT, 20L);
    curl_easy_setopt(curl.get(), CURLOPT_FOLLOWLOCATION, 1L);
    const CURLcode rc = curl_easy_perform(curl.get());
    if (rc != CURLE_OK) throw NetworkError(std::string("Geocoding request failed: ") + curl_easy_strerror(rc));
    long status = 0; curl_easy_getinfo(curl.get(), CURLINFO_RESPONSE_CODE, &status);
    if (status != 200) throw NetworkError("Geocoding service returned HTTP " + std::to_string(status));
    json doc;
    try { doc = json::parse(response); }
    catch (const json::exception& e) { throw ParseError(std::string("Geocoding JSON error: ") + e.what()); }
    if (!doc.is_array() || doc.empty()) throw NetworkError("No matching place found inside the configured map area.");
    const auto& first = doc[0];
    GeoPoint p{std::stod(first.at("lat").get<std::string>()), std::stod(first.at("lon").get<std::string>())};
    if (p.lat < b.south || p.lat > b.north || p.lon < b.west || p.lon > b.east)
        throw NetworkError("That place is outside the configured map area. Expand bbox in config/config.json first.");
    return p;
}


// Overpass JSON shape (out skel):
//   {"elements":[ {"type":"node","id":1,"lat":30.3,"lon":78.0},
//                 {"type":"way","id":9,"nodes":[1,2,3]}, ... ]}
// A way is a polyline of node ids; every consecutive pair becomes one road.
RoadNetwork OverpassClient::parseNetwork(const std::string& text) {
    json doc;
    try {
        doc = json::parse(text);
    } catch (const json::parse_error& e) {
        throw ParseError(std::string("Overpass reply is not valid JSON: ") + e.what());
    }
    if (!doc.contains("elements") || !doc["elements"].is_array()) {
        throw ParseError("Overpass reply has no 'elements' array");
    }

    RoadNetwork net;
    try {
        // Pass 1: remember every node's coordinates by OSM id.
        std::unordered_map<long long, GeoPoint> coords;
        coords.reserve(doc["elements"].size());
        for (const auto& el : doc["elements"]) {
            if (el.value("type", "") == "node" && el.contains("lat") && el.contains("lon")) {
                coords[el.at("id").get<long long>()] = {el.at("lat").get<double>(), el.at("lon").get<double>()};
            }
        }

        // Pass 2: create graph nodes lazily (only nodes that lie on a road) and add edges.
        std::unordered_map<long long, RoadNetwork::NodeId> internal;
        auto resolve = [&](long long osmId) {
            auto it = internal.find(osmId);
            if (it != internal.end()) return it->second;
            const RoadNetwork::NodeId id = net.addNode(coords.at(osmId));
            internal.emplace(osmId, id);
            return id;
        };

        for (const auto& el : doc["elements"]) {
            if (el.value("type", "") != "way" || !el.contains("nodes")) continue;
            const auto& refs = el["nodes"];
            for (std::size_t i = 1; i < refs.size(); ++i) {
                const long long a = refs[i - 1].get<long long>();
                const long long b = refs[i].get<long long>();
                if (a == b || !coords.count(a) || !coords.count(b)) continue;  // clipped at box edge
                net.addRoad(resolve(a), resolve(b));  // weight = Haversine metres
            }
        }
    } catch (const json::exception& e) {
        throw ParseError(std::string("unexpected Overpass JSON structure: ") + e.what());
    }

    if (net.edgeCount() == 0) throw ParseError("Overpass reply contained no roads for this area");
    return net.largestComponent();
}

}  // namespace logi
