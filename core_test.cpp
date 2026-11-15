// core_test.cpp - dependency-free tests for the graph, A* and the simulation.
#include <cassert>
#include <iostream>
#include <queue>
#include <random>

#include "AStarRouter.h"
#include "DeliveryManager.h"
#include "RoadNetwork.h"

using namespace logi;

// Reference implementation to validate A* against.
static double dijkstra(const RoadNetwork& net, RoadNetwork::NodeId s, RoadNetwork::NodeId t) {
    using P = std::pair<double, RoadNetwork::NodeId>;
    std::vector<double> d(net.nodeCount(), 1e300);
    std::priority_queue<P, std::vector<P>, std::greater<P>> q;
    d[s] = 0;
    q.push({0, s});
    while (!q.empty()) {
        auto [du, u] = q.top();
        q.pop();
        if (du > d[u]) continue;
        for (const auto& e : net.neighbors(u))
            if (du + e.second < d[e.first]) {
                d[e.first] = du + e.second;
                q.push({d[e.first], e.first});
            }
    }
    return d[t];
}

static RoadNetwork makeGrid(int n) {
    RoadNetwork net;
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c) net.addNode({30.30 + r * 0.001, 78.03 + c * 0.001});
    auto id = [n](int r, int c) { return static_cast<std::size_t>(r * n + c); };
    std::mt19937 rng(7);
    for (int r = 0; r < n; ++r)
        for (int c = 0; c < n; ++c) {
            if (c + 1 < n && rng() % 6) net.addRoad(id(r, c), id(r, c + 1));
            if (r + 1 < n && rng() % 6) net.addRoad(id(r, c), id(r + 1, c));
        }
    return net.largestComponent();
}

int main() {
    // 1. A* distance equals Dijkstra on many random pairs.
    RoadNetwork net = makeGrid(25);
    AStarRouter router(net);
    std::mt19937 rng(1);
    for (int i = 0; i < 200; ++i) {
        auto a = rng() % net.nodeCount(), b = rng() % net.nodeCount();
        RouteResult r = router.findRoute(a, b);
        assert(r.found);
        assert(std::abs(r.distanceMeters - dijkstra(net, a, b)) < 1e-6);
        assert(r.path.front() == a && r.path.back() == b);
    }
    std::cout << "A* == Dijkstra on 200 random pairs\n";

    // 2. Unreachable target and invalid ids.
    RoadNetwork two;
    auto u = two.addNode({30.0, 78.0}), v = two.addNode({30.1, 78.1});
    assert(!AStarRouter(two).findRoute(u, v).found);
    bool threw = false;
    try { AStarRouter(two).findRoute(0, 99); } catch (const std::out_of_range&) { threw = true; }
    assert(threw);
    threw = false;
    try { two.addRoad(u, u); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    // 3. Full delivery simulation reaches Finished and delivers every parcel.
    DeliveryManager dm(net, 0, 40.0, 200.0);
    dm.addCustomer(net.nodeCount() - 1);
    dm.addCustomer(net.nodeCount() / 2);
    dm.addCustomer(net.nodeCount() / 3);
    dm.start();
    for (int i = 0; i < 200000 && dm.state() != AgentState::Finished; ++i) dm.update(0.05);
    assert(dm.state() == AgentState::Finished);
    assert(dm.deliveredCount() == 3);
    std::cout << "Simulation finished: " << dm.deliveredCount() << " parcels, "
              << dm.totalPlannedMeters() << " m planned\n";

    // 4. No customers => RoutingError (not UB).
    DeliveryManager empty(net, 0);
    threw = false;
    try { empty.start(); } catch (const std::exception&) { threw = true; }
    assert(threw);

    std::cout << "ALL CORE TESTS PASSED\n";
}
