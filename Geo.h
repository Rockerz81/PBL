// Geo.h - geodesy helpers. Header-only so the A* heuristic can be inlined.
#pragma once
#include <algorithm>
#include <cmath>

#include "Types.h"

namespace logi {

constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

inline double toRadians(double deg) { return deg * kPi / 180.0; }
inline double toDegrees(double rad) { return rad * 180.0 / kPi; }

// Great-circle distance between two GPS points (Haversine formula).
//   a = sin^2(dphi/2) + cos(phi1) * cos(phi2) * sin^2(dlambda/2)
//   d = 2 * R * asin(sqrt(a))
// A straight line over the Earth's surface can never be longer than any road
// between the same points, so this is an ADMISSIBLE heuristic for A*.
inline double haversineMeters(const GeoPoint& a, const GeoPoint& b) {
    const double phi1 = toRadians(a.lat);
    const double phi2 = toRadians(b.lat);
    const double dPhi = toRadians(b.lat - a.lat);
    const double dLambda = toRadians(b.lon - a.lon);
    const double s1 = std::sin(dPhi / 2.0);
    const double s2 = std::sin(dLambda / 2.0);
    const double h = s1 * s1 + std::cos(phi1) * std::cos(phi2) * s2 * s2;
    return 2.0 * kEarthRadiusMeters * std::asin(std::min(1.0, std::sqrt(h)));
}

}  // namespace logi
