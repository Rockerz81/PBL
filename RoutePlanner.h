#pragma once
#include "RoadNetwork.h"
#include "dijkstra.h"   // your existing routing logic

// ABSTRACT base class: defines WHAT a route planner must do,
// not HOW. Different algorithms can be plugged in later.
class RoutePlanner {
public:
    virtual ~RoutePlanner() = default;
    virtual void planLeg(int from, int to, const RoadNetwork& network) = 0;  // pure virtual
    virtual std::string getAlgorithmName() const = 0;
};

// CONCRETE class: implements the planner using your Dijkstra code.
class DijkstraPlanner : public RoutePlanner {
public:
    void planLeg(int from, int to, const RoadNetwork& network) override {
        findShortestPath(from, to, network.getAdjacency(), network.getLocationNames());
    }
    std::string getAlgorithmName() const override { return "Dijkstra's Algorithm"; }
};
