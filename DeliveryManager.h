// DeliveryManager.h
// Owns the delivery agent (a single truck): its state machine, the ordered list
// of customers, the A* plan for every leg, and the time-stepped simulation.
//
// The manager is independent of any graphics library - the Raylib frontend only
// READS its state (truckPose(), routeNodes(), ...). That separation keeps the
// simulation unit-testable without opening a window.
#pragma once
#include <cstddef>
#include <vector>

#include "AStarRouter.h"
#include "RoadNetwork.h"

namespace logi {

enum class AgentState { Idle, EnRoute, Servicing, Finished };
const char* toString(AgentState s) noexcept;

class DeliveryManager {
public:
    using NodeId = RoadNetwork::NodeId;

    // Where the truck is and which segment it is on (frontend derives heading).
    struct TruckPose {
        GeoPoint position;
        GeoPoint segmentFrom;
        GeoPoint segmentTo;
    };

    // speedKmh: truck speed. timeScale: simulation speed-up (e.g. 25 = 25x real time).
    // Throws std::out_of_range (bad depot) / std::invalid_argument (bad numbers).
    DeliveryManager(const RoadNetwork& network, NodeId depot, double speedKmh = 40.0,
                    double timeScale = 25.0);

    // --- configuration (any change invalidates the current plan) ---
    void addCustomer(NodeId node);
    void clearCustomers();
    void reorderCustomersNearestFirst();  // greedy nearest-neighbour ordering

    // --- planning & control ---
    void planRoute();   // runs A* for depot->c1->...->cn->depot; throws RoutingError
    void start();       // plans if needed, then begins/resumes driving
    void togglePause();
    void reset();       // back to Idle at the depot, keeps customers
    void update(double dtSeconds);  // advance simulation by real seconds

    // --- read-only state ---
    AgentState state() const noexcept { return state_; }
    bool paused() const noexcept { return paused_; }
    bool planned() const noexcept { return planned_; }
    NodeId depot() const noexcept { return depot_; }
    const std::vector<NodeId>& customers() const noexcept { return customers_; }
    std::size_t deliveredCount() const noexcept { return delivered_; }
    double totalPlannedMeters() const noexcept { return totalMeters_; }
    std::size_t totalNodesExpanded() const noexcept { return nodesExpanded_; }
    double timeScale() const noexcept { return timeScale_; }
    void setTimeScale(double scale);

    const std::vector<NodeId>& routeNodes() const noexcept { return fullRoute_; }  // for drawing
    TruckPose truckPose() const;

private:
    struct Leg {
        std::vector<NodeId> path;
        std::vector<double> cumulative;  // metres from leg start to path[i]; cumulative[0] = 0
        double length{0.0};
    };

    TruckPose poseOnLeg(const Leg& leg, double distance) const;
    void arriveAtStop();
    void beginNextLeg();

    const RoadNetwork& network_;
    AStarRouter router_;
    NodeId depot_;
    std::vector<NodeId> customers_;

    std::vector<Leg> legs_;
    std::vector<NodeId> fullRoute_;
    bool planned_{false};
    double totalMeters_{0.0};
    std::size_t nodesExpanded_{0};

    AgentState state_{AgentState::Idle};
    bool paused_{false};
    std::size_t legIndex_{0};
    double distanceOnLeg_{0.0};
    double dwellRemaining_{0.0};
    std::size_t delivered_{0};

    double speedMps_;
    double timeScale_;
    static constexpr double kDwellSeconds = 1.2;  // real seconds spent "handing over" a parcel
};

}  // namespace logi
