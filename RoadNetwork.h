// RoadNetwork.h
// Encapsulated road graph: locations (nodes with GPS coordinates) joined by
// weighted roads (edges, weight = metres).
//
// Storage is a SPARSE ADJACENCY LIST: adjacency_[u] is a vector of
// (neighbour, weight) pairs. Memory is O(V + E), unlike a V x V matrix, which is
// what makes city-sized OpenStreetMap graphs (tens of thousands of nodes) viable.
#pragma once
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#include "Types.h"

namespace logi {

class RoadNetwork {
public:
    using NodeId = std::size_t;
    using Adjacent = std::pair<NodeId, double>;  // (neighbour id, road length in metres)
    static constexpr NodeId kInvalid = std::numeric_limits<NodeId>::max();

    // --- mutation (the only way to change the graph; data members are private) ---
    NodeId addNode(const GeoPoint& position);

    // Adds an UNDIRECTED road. If meters < 0 the Haversine length is used.
    // The weight is clamped to be >= the straight-line distance so the A*
    // heuristic stays admissible no matter what the caller passes.
    // Throws std::out_of_range for bad ids, std::invalid_argument for self loops.
    void addRoad(NodeId a, NodeId b, double meters = -1.0);

    // --- read-only queries (all const) ---
    std::size_t nodeCount() const noexcept { return positions_.size(); }
    std::size_t edgeCount() const noexcept { return edgeCount_; }
    bool empty() const noexcept { return positions_.empty(); }
    bool contains(NodeId id) const noexcept { return id < positions_.size(); }

    const GeoPoint& position(NodeId id) const;                 // throws std::out_of_range
    const std::vector<Adjacent>& neighbors(NodeId id) const;   // throws std::out_of_range
    bool hasRoad(NodeId a, NodeId b) const;

    NodeId nearestNode(const GeoPoint& p) const;  // O(V) scan; throws if empty
    BoundingBox bounds() const;                   // throws if empty

    // Returns a compacted copy containing only the biggest connected component.
    // OSM extracts contain isolated fragments; removing them guarantees that any
    // two remaining nodes have a route.
    RoadNetwork largestComponent() const;

private:
    void checkId(NodeId id) const;

    std::vector<GeoPoint> positions_;
    std::vector<std::vector<Adjacent>> adjacency_;
    std::size_t edgeCount_{0};
};

}  // namespace logi
