// AStarRouter.h - A* shortest-path search over a RoadNetwork. Header-only.
//
//   f(n) = g(n) + h(n)
//     g(n) = cheapest known road distance from start to n
//     h(n) = Haversine (straight-line GPS) distance from n to the goal
//
// Correctness: h never overestimates (admissible) and satisfies the triangle
// inequality (consistent), so the first time the goal is popped from the heap
// its g-value is optimal and every node is expanded at most once.
#pragma once
#include <algorithm>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

#include "Geo.h"
#include "RoadNetwork.h"

namespace logi {

struct RouteResult {
    bool found{false};
    double distanceMeters{0.0};
    std::vector<RoadNetwork::NodeId> path;  // start ... goal (inclusive)
    std::size_t nodesExpanded{0};           // useful to show A* beating Dijkstra
};

class AStarRouter {
public:
    using NodeId = RoadNetwork::NodeId;

    // Holds a const reference: the network must outlive the router.
    explicit AStarRouter(const RoadNetwork& network) : network_(network) {}

    // Throws std::out_of_range for invalid ids. Returns found=false if unreachable.
    RouteResult findRoute(NodeId start, NodeId goal) const {
        const GeoPoint& goalPos = network_.position(goal);  // validates goal
        network_.position(start);                           // validates start

        constexpr double kInf = std::numeric_limits<double>::infinity();
        const std::size_t n = network_.nodeCount();
        std::vector<double> g(n, kInf);
        std::vector<NodeId> parent(n, RoadNetwork::kInvalid);
        std::vector<char> closed(n, 0);

        // std::priority_queue is a max-heap; the comparator flips it to a MIN-heap on f.
        std::priority_queue<Entry, std::vector<Entry>, Later> open;

        RouteResult result;
        g[start] = 0.0;
        open.push({heuristic(start, goalPos), start});

        while (!open.empty()) {
            const Entry current = open.top();
            open.pop();

            // Lazy deletion: a node may be pushed several times with different f;
            // only the first (smallest) pop is meaningful.
            if (closed[current.node]) continue;
            closed[current.node] = 1;
            ++result.nodesExpanded;

            // EARLY STOP: with a consistent heuristic the goal's g is final now.
            if (current.node == goal) {
                result.found = true;
                result.distanceMeters = g[goal];
                for (NodeId at = goal; at != RoadNetwork::kInvalid; at = parent[at]) {
                    result.path.push_back(at);
                }
                std::reverse(result.path.begin(), result.path.end());
                return result;
            }

            for (const auto& edge : network_.neighbors(current.node)) {
                const NodeId next = edge.first;
                if (closed[next]) continue;
                const double tentative = g[current.node] + edge.second;
                if (tentative < g[next]) {  // found a cheaper way to `next`
                    g[next] = tentative;
                    parent[next] = current.node;
                    open.push({tentative + heuristic(next, goalPos), next});
                }
            }
        }
        return result;  // heap exhausted: goal unreachable
    }

private:
    struct Entry {
        double f;
        NodeId node;
    };
    struct Later {  // "a has lower priority than b" <=> a.f > b.f  => min-heap
        bool operator()(const Entry& a, const Entry& b) const { return a.f > b.f; }
    };

    double heuristic(NodeId n, const GeoPoint& goalPos) const {
        return haversineMeters(network_.position(n), goalPos);
    }

    const RoadNetwork& network_;
};

}  // namespace logi
