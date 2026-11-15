#include "DeliveryManager.h"

#include <algorithm>
#include <stdexcept>

#include "Exceptions.h"
#include "Geo.h"

namespace logi {

const char* toString(AgentState s) noexcept {
    switch (s) {
        case AgentState::Idle: return "Idle";
        case AgentState::EnRoute: return "En route";
        case AgentState::Servicing: return "Delivering parcel";
        case AgentState::Finished: return "Finished (back at depot)";
    }
    return "?";
}

DeliveryManager::DeliveryManager(const RoadNetwork& network, NodeId depot, double speedKmh,
                                 double timeScale)
    : network_(network),
      router_(network),
      depot_(depot),
      speedMps_(speedKmh * 1000.0 / 3600.0),
      timeScale_(timeScale) {
    network_.position(depot);  // throws std::out_of_range if invalid
    if (!(speedKmh > 0.0) || !(timeScale > 0.0)) {
        throw std::invalid_argument("DeliveryManager: speed and timeScale must be > 0");
    }
}

void DeliveryManager::setTimeScale(double scale) {
    if (!(scale > 0.0)) throw std::invalid_argument("timeScale must be > 0");
    timeScale_ = scale;
}

void DeliveryManager::addCustomer(NodeId node) {
    network_.position(node);  // validates
    customers_.push_back(node);
    reset();
    legs_.clear();
    fullRoute_.clear();
    planned_ = false;
}

void DeliveryManager::clearCustomers() {
    customers_.clear();
    reset();
    legs_.clear();
    fullRoute_.clear();
    planned_ = false;
}

void DeliveryManager::reorderCustomersNearestFirst() {
    std::vector<NodeId> remaining = customers_;
    std::vector<NodeId> ordered;
    NodeId current = depot_;
    while (!remaining.empty()) {
        auto best = std::min_element(remaining.begin(), remaining.end(), [&](NodeId a, NodeId b) {
            return haversineMeters(network_.position(current), network_.position(a)) <
                   haversineMeters(network_.position(current), network_.position(b));
        });
        current = *best;
        ordered.push_back(current);
        remaining.erase(best);
    }
    customers_ = std::move(ordered);
    reset();
    legs_.clear();
    fullRoute_.clear();
    planned_ = false;
}

void DeliveryManager::planRoute() {
    if (customers_.empty()) throw RoutingError("no customers to deliver to");

    // Stops: depot -> customers in the given order -> depot.
    std::vector<NodeId> stops;
    stops.push_back(depot_);
    stops.insert(stops.end(), customers_.begin(), customers_.end());
    stops.push_back(depot_);

    std::vector<Leg> legs;
    std::vector<NodeId> full;
    double total = 0.0;
    std::size_t expanded = 0;

    for (std::size_t i = 0; i + 1 < stops.size(); ++i) {
        const RouteResult r = router_.findRoute(stops[i], stops[i + 1]);
        if (!r.found) {
            throw RoutingError("no road route between stop " + std::to_string(i) + " and " +
                               std::to_string(i + 1));
        }
        Leg leg;
        leg.path = r.path;
        leg.cumulative.assign(leg.path.size(), 0.0);
        for (std::size_t k = 1; k < leg.path.size(); ++k) {
            leg.cumulative[k] = leg.cumulative[k - 1] + haversineMeters(network_.position(leg.path[k - 1]),
                                                                        network_.position(leg.path[k]));
        }
        leg.length = leg.cumulative.back();
        total += r.distanceMeters;
        expanded += r.nodesExpanded;
        full.insert(full.end(), leg.path.begin(), leg.path.end());
        legs.push_back(std::move(leg));
    }

    // Commit only after every leg succeeded (strong exception guarantee).
    legs_ = std::move(legs);
    fullRoute_ = std::move(full);
    totalMeters_ = total;
    nodesExpanded_ = expanded;
    planned_ = true;
    reset();
}

void DeliveryManager::start() {
    if (!planned_) planRoute();
    if (state_ == AgentState::Finished) reset();
    if (state_ == AgentState::Idle) {
        state_ = AgentState::EnRoute;
        legIndex_ = 0;
        distanceOnLeg_ = 0.0;
    }
    paused_ = false;
}

void DeliveryManager::togglePause() {
    if (state_ == AgentState::Idle || state_ == AgentState::Finished) return;
    paused_ = !paused_;
}

void DeliveryManager::reset() {
    state_ = AgentState::Idle;
    paused_ = false;
    legIndex_ = 0;
    distanceOnLeg_ = 0.0;
    dwellRemaining_ = 0.0;
    delivered_ = 0;
}

void DeliveryManager::update(double dt) {
    if (paused_ || dt <= 0.0) return;
    switch (state_) {
        case AgentState::EnRoute: {
            const Leg& leg = legs_[legIndex_];
            distanceOnLeg_ += speedMps_ * timeScale_ * dt;
            if (distanceOnLeg_ >= leg.length) {
                distanceOnLeg_ = leg.length;
                arriveAtStop();
            }
            break;
        }
        case AgentState::Servicing:
            dwellRemaining_ -= dt;
            if (dwellRemaining_ <= 0.0) beginNextLeg();
            break;
        case AgentState::Idle:
        case AgentState::Finished:
            break;
    }
}

void DeliveryManager::arriveAtStop() {
    const bool lastLeg = (legIndex_ + 1 == legs_.size());  // last leg returns to depot
    if (lastLeg) {
        state_ = AgentState::Finished;
    } else {
        ++delivered_;  // sequential drop-off completed
        state_ = AgentState::Servicing;
        dwellRemaining_ = kDwellSeconds;
    }
}

void DeliveryManager::beginNextLeg() {
    ++legIndex_;
    distanceOnLeg_ = 0.0;
    state_ = AgentState::EnRoute;
}

DeliveryManager::TruckPose DeliveryManager::truckPose() const {
    if (legs_.empty()) {
        const GeoPoint& d = network_.position(depot_);
        return {d, d, d};
    }
    return poseOnLeg(legs_[std::min(legIndex_, legs_.size() - 1)], distanceOnLeg_);
}

DeliveryManager::TruckPose DeliveryManager::poseOnLeg(const Leg& leg, double distance) const {
    if (leg.path.size() < 2) {
        const GeoPoint& p = network_.position(leg.path.front());
        return {p, p, p};
    }
    const double s = std::clamp(distance, 0.0, leg.length);
    // First cumulative value strictly greater than s => the segment end index.
    const auto it = std::upper_bound(leg.cumulative.begin(), leg.cumulative.end(), s);
    const std::size_t hi =
        std::clamp<std::size_t>(static_cast<std::size_t>(it - leg.cumulative.begin()), 1, leg.path.size() - 1);
    const std::size_t lo = hi - 1;
    const double span = leg.cumulative[hi] - leg.cumulative[lo];
    const double t = span > 0.0 ? std::clamp((s - leg.cumulative[lo]) / span, 0.0, 1.0) : 0.0;

    const GeoPoint& a = network_.position(leg.path[lo]);
    const GeoPoint& b = network_.position(leg.path[hi]);
    return {{a.lat + (b.lat - a.lat) * t, a.lon + (b.lon - a.lon) * t}, a, b};  // linear interpolation
}

}  // namespace logi
