#pragma once
#include <vector>
#include <string>
#include <utility>
#include <stdexcept>

// Represents the city map: locations (nodes) and roads (weighted edges).
// The adjacency list is PRIVATE; outside code can only use the public methods.
class RoadNetwork {
private:
    std::vector<std::string> locationNames;
    std::vector<std::vector<std::pair<int, int>>> adjacency;

    bool isValid(int id) const { return id >= 0 && id < (int)locationNames.size(); }

public:
    // Adds a location and returns its index (id)
    int addLocation(const std::string& name) {
        locationNames.push_back(name);
        adjacency.emplace_back();
        return (int)locationNames.size() - 1;
    }

    // Adds a two-way (undirected) road with distance in km
    void addRoad(int u, int v, int weightKm) {
        if (!isValid(u) || !isValid(v) || weightKm <= 0)
            throw std::invalid_argument("Invalid road definition");
        adjacency[u].push_back({v, weightKm});
        adjacency[v].push_back({u, weightKm});
    }

    int size() const { return (int)locationNames.size(); }

    const std::string& getName(int id) const {
        if (!isValid(id)) throw std::out_of_range("Invalid location id");
        return locationNames[id];
    }

    // Read-only access, used by the routing algorithm
    const std::vector<std::string>& getLocationNames() const { return locationNames; }
    const std::vector<std::vector<std::pair<int, int>>>& getAdjacency() const { return adjacency; }
};
