#include "RoadNetwork.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <stdexcept>
#include <string>

#include "Geo.h"

namespace logi {

void RoadNetwork::checkId(NodeId id) const {
    if (id >= positions_.size()) {
        throw std::out_of_range("RoadNetwork: node id " + std::to_string(id) + " out of range");
    }
}

RoadNetwork::NodeId RoadNetwork::addNode(const GeoPoint& position) {
    if (!std::isfinite(position.lat) || !std::isfinite(position.lon)) {
        throw std::invalid_argument("RoadNetwork: non-finite coordinate");
    }
    positions_.push_back(position);
    adjacency_.emplace_back();
    return positions_.size() - 1;
}

void RoadNetwork::addRoad(NodeId a, NodeId b, double meters) {
    checkId(a);
    checkId(b);
    if (a == b) throw std::invalid_argument("RoadNetwork: self-loop roads are not allowed");

    const double straight = haversineMeters(positions_[a], positions_[b]);
    if (meters < 0.0) meters = straight;
    if (!std::isfinite(meters)) throw std::invalid_argument("RoadNetwork: non-finite weight");
    meters = std::max(meters, straight);  // keep h(n) admissible

    if (hasRoad(a, b)) return;  // OSM ways share segments; ignore duplicates
    adjacency_[a].emplace_back(b, meters);
    adjacency_[b].emplace_back(a, meters);
    ++edgeCount_;
}

const GeoPoint& RoadNetwork::position(NodeId id) const {
    checkId(id);
    return positions_[id];
}

const std::vector<RoadNetwork::Adjacent>& RoadNetwork::neighbors(NodeId id) const {
    checkId(id);
    return adjacency_[id];
}

bool RoadNetwork::hasRoad(NodeId a, NodeId b) const {
    checkId(a);
    checkId(b);
    const auto& list = adjacency_[a];  // degree is tiny (<= ~6), linear scan is fine
    return std::any_of(list.begin(), list.end(), [b](const Adjacent& e) { return e.first == b; });
}

RoadNetwork::NodeId RoadNetwork::nearestNode(const GeoPoint& p) const {
    if (positions_.empty()) throw std::logic_error("RoadNetwork: nearestNode on empty network");
    NodeId best = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    for (NodeId i = 0; i < positions_.size(); ++i) {
        const double d = haversineMeters(p, positions_[i]);
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    return best;
}

BoundingBox RoadNetwork::bounds() const {
    if (positions_.empty()) throw std::logic_error("RoadNetwork: bounds of empty network");
    BoundingBox b{positions_[0].lat, positions_[0].lon, positions_[0].lat, positions_[0].lon};
    for (const auto& p : positions_) {
        b.south = std::min(b.south, p.lat);
        b.north = std::max(b.north, p.lat);
        b.west = std::min(b.west, p.lon);
        b.east = std::max(b.east, p.lon);
    }
    return b;
}

RoadNetwork RoadNetwork::largestComponent() const {
    const std::size_t n = positions_.size();
    if (n == 0) return {};

    // Label connected components with BFS.
    std::vector<NodeId> label(n, kInvalid);
    std::vector<std::size_t> sizes;
    for (NodeId s = 0; s < n; ++s) {
        if (label[s] != kInvalid) continue;
        const NodeId c = sizes.size();
        sizes.push_back(0);
        std::queue<NodeId> q;
        q.push(s);
        label[s] = c;
        while (!q.empty()) {
            const NodeId u = q.front();
            q.pop();
            ++sizes[c];
            for (const auto& edge : adjacency_[u]) {
                const NodeId v = edge.first;
                if (label[v] == kInvalid) {
                    label[v] = c;
                    q.push(v);
                }
            }
        }
    }
    const NodeId best = static_cast<NodeId>(std::max_element(sizes.begin(), sizes.end()) - sizes.begin());

    // Rebuild with compact ids.
    RoadNetwork out;
    std::vector<NodeId> remap(n, kInvalid);
    for (NodeId i = 0; i < n; ++i) {
        if (label[i] == best) remap[i] = out.addNode(positions_[i]);
    }
    for (NodeId u = 0; u < n; ++u) {
        if (remap[u] == kInvalid) continue;
        for (const auto& edge : adjacency_[u]) {
            if (u < edge.first) out.addRoad(remap[u], remap[edge.first], edge.second);
        }
    }
    return out;
}

}  // namespace logi
